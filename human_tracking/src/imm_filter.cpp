#include "human_tracking/imm_filter.hpp"

namespace vk {

IMMFilter::IMMFilter(ros::NodeHandle *nh) {
    m_numberModels = 3;
    setprobability = 0.7;
    m_immVisualization = true;
    m_horizonal = 5;
    for (unsigned int i = 0; i < m_numberModels; i++) {
        double checksum = 0.0;
        for(unsigned int j = 0; j < m_numberModels; j++) {
            if(i != j) {
                m_markovTransitionProbabilities(i, j) = (1.0 - setprobability)/(m_numberModels - 1);
            }else {
                m_markovTransitionProbabilities(i, j) = setprobability;
            }
            checksum += m_markovTransitionProbabilities(i, j);
        }
        if(abs(checksum - 1.0) > 0.01) {
            ROS_ERROR("m_markovTransitionProbabilities have to sum up one!");
        }
    }

    for(unsigned int i = 0; i < m_numberModels; i++) {
        unsigned int MODEL_IDX = i;
        EKF::Ptr ekf (new EKF(MODEL_IDX));
        m_kalmanFilters.push_back(ekf);
    }
}

FilterState::Ptr IMMFilter::initializeTrackState(Observation::ConstPtr observation, const VelocityVector& initialVelocity) {
    IMMState::Ptr immState (new IMMState);
    for(EKF::Ptr ekf:m_kalmanFilters) {
        IMMHypothesis::Ptr hypothesis (new IMMHypothesis);
        ekf->initializeTrackState(hypothesis, observation, initialVelocity);
        hypothesis->m_probability = 1.0/(double)m_numberModels;
        immState->m_hypotheses.push_back(hypothesis);
    }
    immState->m_currentHypothesis = immState->m_hypotheses.front();
    immState->m_currentHypothesisIdx = 0;
    updateStateEstimate(immState);

    for(int i = 0; i < m_horizonal; i++) {
        StateVector state = StateVector::Zero();
        immState->m_statePrediction.push_back(state);
    }
    return immState;
}

void IMMFilter::updateStateEstimate(IMMState::Ptr state) {
    state->m_x = StateVector::Zero();
    for(int i = 0; i < state->m_hypotheses.size(); i++) {
        IMMHypothesis::Ptr hyp = state->m_hypotheses.at(i);
        state->m_x += hyp->m_probability * hyp->m_x;
    }

    state->m_C = StateMatrix::Zero();
    for(int i = 0; i < state->m_hypotheses.size(); i++) {
        IMMHypothesis::Ptr hyp = state->m_hypotheses.at(i);
        StateVector diff = hyp->m_x - state->m_x;
        state->m_C += hyp->m_probability * (hyp->m_C + (diff * diff.transpose()));
    }
}

void IMMFilter::computeMixingProbabilities(IMMState::Ptr state) {
    state->m_mixingProbabilities = IMMMatrix::Zero();
    for(int i = 0; i < state->m_hypotheses.size(); i++) {
        double normalizer = 0.0;
        for(int l = 0; l < state->m_hypotheses.size(); l++) {
            IMMHypothesis::Ptr hyp = state->m_hypotheses.at(l);
            normalizer += m_markovTransitionProbabilities(l, i) * hyp->m_probability;
        }
        for(int j = 0; j < state->m_hypotheses.size(); j++) {
            state->m_mixingProbabilities(j, i) = m_markovTransitionProbabilities(j, i) * state->m_hypotheses.at(j)->m_probability / normalizer;
        }
    }
}

void IMMFilter::computeMixedMeanAndCovariance(IMMState::Ptr state) {
    for(int i = 0; i < state->m_hypotheses.size(); i++) {
        state->m_hypotheses.at(i)->m_xMixed = StateVector::Zero();
        state->m_hypotheses.at(i)->m_CMixed = StateMatrix::Zero();
        for(int j = 0; j < state->m_hypotheses.size(); j++) {
            state->m_hypotheses.at(i)->m_xMixed += state->m_mixingProbabilities(j, i) * state->m_hypotheses.at(j)->x();
        }
        for(int j = 0; j < state->m_hypotheses.size(); j++) {
            StateVector diff = state->m_hypotheses.at(j)->x() - state->m_hypotheses.at(i)->xMixed();
            state->m_hypotheses.at(i)->m_CMixed += state->m_mixingProbabilities(j, i) * (state->m_hypotheses.at(j)->C() + diff * diff.transpose());
        }
    }
}

void IMMFilter::predictTrackState(FilterState::Ptr state, double T) {
    IMMState::Ptr imm = boost::dynamic_pointer_cast<IMMState>(state);
    computeMixingProbabilities(imm);
    computeMixedMeanAndCovariance(imm);
    for(int i = 0; i < imm->m_hypotheses.size(); i++ ) {
        imm->m_hypotheses.at(i)->setMixedValuesForState();
    }
    for(int i = 0; i < m_kalmanFilters.size(); i++) {
        m_kalmanFilters.at(i)->predictTrackState(imm->m_hypotheses.at(i), T);
    }
    mixPredictions(imm);
}      

void IMMFilter::mixPredictions(IMMState::Ptr state) {
    state->m_xp = StateVector::Zero();
    for(int i = 0; i < state->m_hypotheses.size(); i++) {
        IMMHypothesis::Ptr hyp = state->m_hypotheses.at(i);
        state->m_xp += hyp->m_xp * hyp->m_probability;
    }

    state->m_Cp = StateMatrix::Zero();
    for(int i = 0; i < state->m_hypotheses.size(); i++) {
        IMMHypothesis::Ptr hyp = state->m_hypotheses.at(i);
        StateVector diff = hyp->m_xp - state->m_xp;
        state->m_Cp += hyp->m_probability * (hyp->m_Cp + (diff * diff.transpose()));
    }
}

void IMMFilter::updateMatchedTrack(FilterState::Ptr state, Observation::ConstPtr observation) {
    IMMState::Ptr imm = boost::dynamic_pointer_cast<IMMState>(state);
    for(int i = 0; i < m_kalmanFilters.size(); i++) {
        IMMHypothesis::Ptr hyp = imm->m_hypotheses.at(i);
        m_kalmanFilters.at(i)->updateMatchedTrack(hyp, observation);
    }
    modeProbabilityUpdate(imm, observation);
    updateStateEstimate(imm);
}

double IMMFilter::normalDistribution(ObsVector& z, ObsVector& z_hat, ObsMatrix& S) {
    double a = 1.0 *sqrt(S.determinant()) / pow(2*M_PI, OBS_DIM/2);
    ObsVector diff = z - z_hat;
    return a * exp(-0.5 * diff.transpose() * S.inverse() * diff);
}

void IMMFilter::modeProbabilityUpdate(FilterState::Ptr state, Observation::ConstPtr observation) {
    IMMState::Ptr imm = boost::dynamic_pointer_cast<IMMState>(state);
    ObsVector z = observation->z;
    for(int i = 0; i < imm->m_hypotheses.size(); i++) {
        IMMHypothesis::Ptr hyp = imm->m_hypotheses.at(i);
        ObsMatrix S = hyp->m_H * hyp->m_Cp * hyp->m_H.transpose() + observation->R;
        ObsVector z_hat = hyp->m_H * hyp->m_xp;
        double prob_i = 0.0;
        for(int j = 0; j < imm->m_hypotheses.size(); j++) {
            prob_i += m_markovTransitionProbabilities(j, i) * imm->m_hypotheses.at(j)->m_probability;
        }
        double prob = 0.0;
        for(int l = 0; l < imm->m_hypotheses.size(); l++) {
            double prob_l = 0;
            for(int j = 0; j < imm->m_hypotheses.size(); j++) {
                prob_l += m_markovTransitionProbabilities(j, l) * imm->m_hypotheses.at(j)->m_probability;
            }
            ObsVector z_hat_l = imm->m_hypotheses.at(l)->m_H * imm->m_hypotheses.at(l)->m_xp;
            ObsMatrix S_l = imm->m_hypotheses.at(l)->m_H * imm->m_hypotheses.at(l)->m_Cp * imm->m_hypotheses.at(l)->m_H.transpose() + observation->R;
            prob += normalDistribution(z, z_hat_l, S_l) * prob_l;
        }
        hyp->m_probability = normalDistribution(z, z_hat, S) * prob_i/prob;
    }
}

void IMMFilter::updateCurrentHypothesis(IMMState::Ptr state) {
    double bestHypothesis = 0.0;
    unsigned int bestHypothesisIdx = 0;
    for (unsigned int i = 0; i < state->m_hypotheses.size(); i++) {
        if(state->m_hypotheses.at(i)->m_probability > bestHypothesis) {
            bestHypothesis = state->m_hypotheses.at(i)->m_probability;
            bestHypothesisIdx = i;
        }
    }
    state->m_currentHypothesis = state->m_hypotheses.at(bestHypothesisIdx);
    state->m_currentHypothesisIdx = bestHypothesisIdx;
}

void IMMFilter::statePredictionForHorizonal(FilterState::Ptr state, double deltatime) {
    IMMState::Ptr imm = boost::dynamic_pointer_cast<IMMState>(state);
    for(int i = 0; i < imm->m_hypotheses.size(); i++) {
        imm->m_hypotheses.at(i)->setStateValue();
    }
    for(int i = 0; i < m_horizonal; i++) {
        for(int j = 0; j < m_kalmanFilters.size(); j++) {
            m_kalmanFilters.at(j)->statePrediction(imm->m_hypotheses.at(j), deltatime);
        }
        for(int j = 0; j < imm->m_hypotheses.size(); j++) {
            imm->m_hypotheses.at(j)->setPredictValue();
        }
        imm->m_x_ = StateVector::Zero();
        for(int j = 0; j < imm->m_hypotheses.size(); j++) {
            imm->m_x_ += imm->m_hypotheses.at(j)->m_probability * imm->m_hypotheses.at(j)->m_x_;
        }
        imm->m_statePrediction.at(i) = imm->m_x_;
    }
}

}