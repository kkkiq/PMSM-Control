function [overshoot, riseTime, settlingTime, ssError, mse, uMax, uInt] =...
          computeKPI(timeVec, stateVec, controlVec, setPoint)
% Take mean of final values (filter noisy states)
    stateEndEstimate = mean(stateVec(end-10:end));
% Rise time[s] (0 to 100% of final value):
    i = 1;
    while stateVec(i) < stateEndEstimate
        i = i+1;
    end
    riseTime = timeVec(i);

% Overshoot[%] (peak amplitude):    
    overshoot = ((max(stateVec) - setPoint(end)) / setPoint(end)) * 100;
    if overshoot < 0
        overshoot = 0;
    end

% Settling time (2% criteria) -> 2% of final value:
    i = numel(stateVec);

    ub = stateEndEstimate*0.98;
    lb = stateEndEstimate*1.02;
    while (stateVec(i) > ub) && (stateVec(i) < lb)
        if i > 1
            i = i-1;
        else
            break
        end
    end
    settlingTime = timeVec(i);

% Steady-State error:
    ssError = abs(stateEndEstimate - setPoint(end));

% MSE
    if numel(stateVec) > numel(setPoint)
        setPoint = ones(numel(stateVec),1)*setPoint;
    end
    mse = immse(stateVec, setPoint);

% Control signal integral (|u|)
    uInt = sum(abs(controlVec));

% Maximum control input(module)
    uMax = max(abs(controlVec));
end