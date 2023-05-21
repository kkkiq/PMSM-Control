function [stateDerivative, state] = linearizePMSMSim(Input, Xop, simParam)
    simParam.Input = [zeros(numel(Input(1)),1) Input(1) Input(2)];
    simParam.xInitial = Xop;
    assignin("base","simParam",simParam);
    simData = sim(simParam.Model);
    stateDerivative = simData.stateDerivative(end, :);
    state = simData.state(end, :);
end