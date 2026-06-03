%% full_arm_new_algorithm_normalized_distance_showcase.m
% Normalized prototype-distance plot + instruction markers + delay indicators.
%
% This version is meant for showcasing only the new method:
%   - Hand/gripper is still classified independently.
%   - Body/arm state uses the current multi-channel body prototype classifier.
%   - Deltoid arm-down is handled by a separate zero-feature override.
%   - HEIGHT_DOWN is excluded from the normal prototype classifier unless the
%     deltoid zero-feature override is active.
%   - Chest rotation direction changes only after a confirmed 1 s release.
%   - Biceps/triceps robot operation is reversed: biceps = -Distance,
%     triceps = +Distance.
%   - No old body-method plots or printed comparison are produced.
%
% Expected timing/label columns:
%   timestamp, instruction, expected_muscle, expected_state, test_phase
%
% For the showcase plot, the script displays the normalized prototype distances
% used by the new body classifier. Smaller distance means the current EMG feature
% vector is closer to that prototype/class.
%aa
% The new classifier uses the same feature columns:
%   rms_*, mav_*, wl_*
% or pred_* when available for hand percentage reconstruction.

clear; clc; close all;

%% =========================
% Showcase plot option
%% =========================
% Choose what the first/main figure shows:
%   "raw"                 : raw channel values if logged; otherwise WL/RMS/MAV/pred fallback
%   "energy"              : relative channel energy (%) from RMS, MAV, and WL
%   "distance"            : raw prototype distances to each body class
%   "normalized_distance" : normalized prototype distances to each body class
SHOWCASE_PLOT_MODE = "energy";

% Current Python-algorithm parameters mirrored in this MATLAB analysis script.
USE_DELTOID_ZERO_ARM_DOWN_OVERRIDE = true;
DELTOID_ZERO_ARM_DOWN_WINDOW_SEC = 1.2;
DELTOID_ZERO_ARM_DOWN_RATIO = 0.85;
DELTOID_ZERO_EPS = 0.0;

BODY_FEATURE_HISTORY_LEN = 7;
BODY_CONFIRM_SAMPLES = 5;
BODY_CLASSIFIER_MIN_MARGIN = 0.8;
BODY_CLASSIFIER_MIN_STD = 0.20;

% Chest stops immediately when inactive, but the stored rotation direction is
% allowed to switch only after a confirmed release interval.
CHEST_ROTATION_RELEASE_CONFIRM_SEC = 1.0;

%% =========================
% Load CSV
%% =========================
[file, path] = uigetfile('*.csv', 'Select EMG test samples CSV');
if isequal(file, 0)
    error('No file selected.');
end

csvFile = fullfile(path, file);
T = readtable(csvFile, 'TextType', 'string');

if ~hasColumn(T, "timestamp")
    error('CSV must contain a timestamp column.');
end

t = T.timestamp - T.timestamp(1);

allChannels  = ["hand", "biceps", "triceps", "deltoid", "chest"];
bodyChannels = ["biceps", "triceps", "deltoid", "chest"];
features     = ["rms", "mav", "wl"];

instruction    = getStringColumnOrDefault(T, "instruction", "none");
expectedMuscle = getStringColumnOrDefault(T, "expected_muscle", "unknown");
expectedState  = getStringColumnOrDefault(T, "expected_state", "unknown");
testPhase      = getStringColumnOrDefault(T, "test_phase", "capture");

if hasColumn(T, "step_index")
    stepIndex = T.step_index;
else
    stepIndex = buildStepIndexFromInstructions(instruction);
end

captureMask = testPhase == "capture";

%% =========================
% Expected cumulative labels
%% =========================
expectedHandClass = buildExpectedHandClassCumulative(expectedMuscle, expectedState);
expectedBodyClass = buildExpectedBodyClassCumulative(expectedMuscle, expectedState);

validHandExpected = expectedHandClass ~= "UNKNOWN";
validBodyExpected = expectedBodyClass ~= "UNKNOWN";

%% =========================
% Hand classifier
% This is the same independent hand logic used before.
%% =========================
pctForHand = buildIndependentPercentages(T, allChannels, features);
newHandClassRaw = classifyHandIndependent(pctForHand.hand);
newHandClass = holdUnknownStates(newHandClassRaw, "HAND_OPEN");

%% =========================
% New body prototype classifier
%% =========================
[Xbody, bodyFeatureNames] = buildNewFeatureMatrix(T, bodyChannels, BODY_FEATURE_HISTORY_LEN); %#ok<NASGU>

trainMask = captureMask & validBodyExpected;
bodyModel = trainPrototypeClassifier(Xbody, expectedBodyClass, trainMask, BODY_CLASSIFIER_MIN_STD);

% Current algorithm: arm-down is not allowed to be selected by normal body
% prototype closeness. It is forced only by the separate deltoid zero-feature
% detector.
deltoidZeroDown = detectDeltoidZeroArmDown( ...
    T, t, ...
    DELTOID_ZERO_ARM_DOWN_WINDOW_SEC, ...
    DELTOID_ZERO_ARM_DOWN_RATIO, ...
    DELTOID_ZERO_EPS);

if USE_DELTOID_ZERO_ARM_DOWN_OVERRIDE
    excludedActions = "HEIGHT_DOWN";
else
    excludedActions = strings(0,1);
end

[newBodyRaw, bodyConfidence, bodyDistances, bodyClassNames, minConfidenceMargin] = classifyPrototype( ...
    Xbody, bodyModel, BODY_CLASSIFIER_MIN_MARGIN, excludedActions); %#ok<NASGU,ASGLU>

if USE_DELTOID_ZERO_ARM_DOWN_OVERRIDE
    newBodyRaw(deltoidZeroDown) = "HEIGHT_DOWN";
end

newBodyClass = temporalConfirmClasses(newBodyRaw, BODY_CONFIRM_SAMPLES, "BODY_REST");

% Convert accepted body actions to the actual operational command timeline.
% This preserves biceps/triceps direction reversal and the chest rotation
% direction/debounce logic used by the current Python controller.
newBodyOperationClass = bodyActionToOperationTimeline( ...
    t, newBodyClass, CHEST_ROTATION_RELEASE_CONFIRM_SEC);

%% =========================
% Accuracy / response indicators
%% =========================
handEvalMask = captureMask & validHandExpected;
bodyEvalMask = captureMask & validBodyExpected;

newHandAcc = safeAccuracy(newHandClass(handEvalMask), expectedHandClass(handEvalMask));
newBodyAcc = safeAccuracy(newBodyClass(bodyEvalMask), expectedBodyClass(bodyEvalMask));

fprintf('\n===== CURRENT METHOD SHOWCASE =====\n');
fprintf('Hand classifier accuracy:        %.2f %%\n', newHandAcc);
fprintf('Current body prototype accuracy:     %.2f %%\n\n', newBodyAcc);

printConfusion(expectedHandClass(handEvalMask), newHandClass(handEvalMask), "HAND CLASSIFIER");
printConfusion(expectedBodyClass(bodyEvalMask), newBodyClass(bodyEvalMask), "NEW BODY PROTOTYPE METHOD");

[handResponseIdx, handDelayStats] = computeClassResponseIndicators( ...
    t, instruction, expectedHandClass, newHandClass);

[newBodyResponseIdx, newBodyDelayStats] = computeClassResponseIndicators( ...
    t, instruction, expectedBodyClass, newBodyClass);

%% =========================
% Delay distribution histograms
% Uses ONLY the delays that are actually written/drawn on the state timeline.
%% =========================
BIN_WIDTH = 0.24;

handDelays = extractDrawnResponseDelays(t, instruction, handResponseIdx);
bodyDelays = extractDrawnResponseDelays(t, instruction, newBodyResponseIdx);

combinedDelays = [handDelays(:); bodyDelays(:)];

figure('Color','w', 'Name', 'Drawn Delay Distribution Histograms');
tlDelay = tiledlayout(3, 1, ...
    'TileSpacing', 'compact', ...
    'Padding', 'compact');

ax1 = nexttile(tlDelay, 1);
histogram(ax1, handDelays, ...
    'BinWidth', BIN_WIDTH, ...
    'Normalization', 'count');
grid(ax1, 'on'); box(ax1, 'on');
xlabel(ax1, 'Delay (s)');
ylabel(ax1, 'Count');
title(ax1, sprintf('Hand Delay Distribution'));

ax2 = nexttile(tlDelay, 2);
histogram(ax2, bodyDelays, ...
    'BinWidth', BIN_WIDTH, ...
    'Normalization', 'count');
grid(ax2, 'on'); box(ax2, 'on');
xlabel(ax2, 'Delay (s)');
ylabel(ax2, 'Count');
title(ax2, sprintf('Body Delay Distribution'));

ax3 = nexttile(tlDelay, 3);
histogram(ax3, combinedDelays, ...
    'BinWidth', BIN_WIDTH, ...
    'Normalization', 'count');
grid(ax3, 'on'); box(ax3, 'on');
xlabel(ax3, 'Delay (s)');
ylabel(ax3, 'Count');
title(ax3, sprintf('Combined Delay Distribution'));

linkaxes([ax1 ax2 ax3], 'x');
%% =========================
% Combined figure: selected showcase signal on top + state timeline on bottom
%% =========================
showcaseMode = lower(strtrim(string(SHOWCASE_PLOT_MODE)));

switch showcaseMode
    case "raw"
        [Ymain, mainNames] = buildRawSignalMatrix(T, allChannels);
        mainTitle = "Raw Channel Values";
        yLabelText = "Raw channel value";
        forceYLim = false;

    case "energy"
        Ymain = 100 * computeRelativeEnergy(T, allChannels);
        mainNames = upperFirstArray(allChannels);
        mainTitle = "Relative Channel Energy";
        yLabelText = "Relative energy (%)";
        forceYLim = true;
        forcedYLim = [0 100];

    case "distance"
        Ymain = bodyDistances;
        mainNames = "Distance to " + bodyClassNames;
        mainTitle = "Raw Prototype Distances";
        yLabelText = "Prototype distance";
        forceYLim = false;

    case "normalized_distance"
        Ymain = normalizeDistanceColumns(bodyDistances);
        mainNames = "Distance to " + bodyClassNames;
        mainTitle = "Normalized Prototype Distances";
        yLabelText = "Normalized prototype distance";
        forceYLim = true;
        forcedYLim = [0 1];

    otherwise
        error('Invalid SHOWCASE_PLOT_MODE: %s. Use "raw", "energy", "distance", or "normalized_distance".', showcaseMode);
end

figShowcase = figure('Color','w', 'Name', 'Current Algorithm Showcase and State Timeline');
tl = tiledlayout(figShowcase, 2, 1, ...
    'TileSpacing', 'compact', ...
    'Padding', 'compact');

%% -------------------------
% Top plot: selected showcase signal
%% -------------------------
axTop = nexttile(tl, 1);
hold(axTop, 'on'); grid(axTop, 'on'); box(axTop, 'on');

pMain = gobjects(size(Ymain,2),1);
for c = 1:size(Ymain,2)
    pMain(c) = plot(axTop, t, Ymain(:,c), ...
        'LineWidth', 1.6, ...
        'DisplayName', mainNames(c));
end

ylabel(axTop, yLabelText);

if forceYLim
    ylim(axTop, forcedYLim);
end

ylTop = ylim(axTop);

title(axTop, sprintf(['%s | Hand acc = %.1f %% | Body acc = %.1f %% | ', ...
               'Hand avg delay = %.2f s | Body avg delay = %.2f s'], ...
               mainTitle, newHandAcc, newBodyAcc, ...
               handDelayStats.avgDelay, newBodyDelayStats.avgDelay));

% Shaded intervals based on the accepted NEW algorithm states.
% These patches show what the algorithm is currently interpreting,
% not what was commanded.
stateColors = getStateColorsFromDistancePlots(bodyClassNames, gobjects(0,1));
shadeFromLogical(t, newBodyOperationClass == "DISTANCE_MINUS", ylTop, stateColors.ARM_CLOSE);
shadeFromLogical(t, newBodyOperationClass == "DISTANCE_PLUS",  ylTop, stateColors.ARM_EXTEND);
shadeFromLogical(t, newBodyOperationClass == "HEIGHT_UP",      ylTop, stateColors.HEIGHT_UP);
shadeFromLogical(t, newBodyOperationClass == "HEIGHT_DOWN",    ylTop, stateColors.HEIGHT_DOWN);
shadeFromLogical(t, newBodyOperationClass == "CHEST_PLUS",     ylTop, stateColors.CHEST_ACTIVE);
shadeFromLogical(t, newBodyOperationClass == "CHEST_MINUS",    ylTop, stateColors.CHEST_ACTIVE);
shadeFromLogical(t, newHandClass == "HAND_CLOSED",  ylTop, [0.4 0.35 1]);

% Keep main traces visually above the shading.
for c = 1:numel(pMain)
    try
        uistack(pMain(c), 'top');
    catch
    end
end


legend(axTop, 'Location', 'bestoutside','AutoUpdate','off');

% Top plot has the named action/command indicators.
drawInstructionLines(t, instruction, ylTop);

pHandShade = patch(nan, nan, [0.4 0.35 1], 'FaceAlpha', 0.10, 'EdgeColor', 'none', ...
    'DisplayName', 'Algorithm: hand closed');
pArmCloseShade = patch(nan, nan, stateColors.ARM_CLOSE, 'FaceAlpha', 0.10, 'EdgeColor', 'none', ...
    'DisplayName', 'Algorithm: biceps active (-distance)');
pArmExtendShade = patch(nan, nan, stateColors.ARM_EXTEND, 'FaceAlpha', 0.10, 'EdgeColor', 'none', ...
    'DisplayName', 'Algorithm: triceps active (+distance)');
pHeightShade = patch(nan, nan, stateColors.HEIGHT_UP, 'FaceAlpha', 0.10, 'EdgeColor', 'none', ...
    'DisplayName', 'Algorithm: height up/down');
pChestShade = patch(nan, nan, stateColors.CHEST_ACTIVE, 'FaceAlpha', 0.10, 'EdgeColor', 'none', ...
    'DisplayName', 'Algorithm: chest rotation');


%% -------------------------
% Bottom plot: algorithm state timeline
%% -------------------------
axBottom = nexttile(tl, 2);
hold(axBottom, 'on'); grid(axBottom, 'on'); box(axBottom, 'on');

plot(axBottom, t, handClassToLevel(newHandClass), ...
    'LineWidth', 3.8, ...
    'DisplayName', 'New hand state', ...
    'Color', [1 0 0]);

plot(axBottom, t, bodyOperationToLevel(newBodyOperationClass), ...
    'LineWidth', 3.8, ...
    'DisplayName', 'New body operation', ...
    'Color', [0 0 1]);

plot(axBottom, t, handClassToLevel(expectedHandClass), '-.', ...
    'LineWidth', 2.8, ...
    'DisplayName', 'Expected hand state', ...
    'Color', [1 0.7 0]);

plot(axBottom, t, bodyClassToLevel(expectedBodyClass), '-.', ...
    'LineWidth', 2.8, ...
    'DisplayName', 'Expected body state', ...
    'Color', [0.2 0.7 1]);

xlabel(axBottom, 'Time (s)');
ylabel(axBottom, 'State level');

title(axBottom, sprintf(['Current Algorithm State Timeline | Hand avg delay = %.2f s | Hand max delay = %.2f s | ', ...
               'Body avg delay = %.2f s | Body max delay = %.2f s'], ...
    handDelayStats.avgDelay, handDelayStats.maxDelay, ...
    newBodyDelayStats.avgDelay, newBodyDelayStats.maxDelay));

yticks(axBottom, 0:6);
yticklabels(axBottom, ["GRASP", "REST", "-DIST", ...
             "+DIST", "+HEIGHT", "-HEIGHT", ...
             "+-ANGLE"]);
ylim(axBottom, [-0.3 7.4]);
ylBottom = ylim(axBottom);

% Bottom plot has the same vertical command lines, but without repeated text labels.
drawInstructionLinesNoLabels(t, instruction, ylBottom);

% Delay indicators are shown only on the bottom state timeline.
% Labels contain only the measured delay time.
drawClassResponseArrows(t, instruction, newBodyResponseIdx, ylBottom, "", 0.85, [0.10 0.10 0.10]);
drawClassResponseArrows(t, instruction, handResponseIdx,    ylBottom, "", 0.06, [0.35 0.35 0.35]);

legend(axBottom, 'Location', 'bestoutside');

linkaxes([axTop axBottom], 'x');

fprintf('Showcase plot mode: %s.\n', showcaseMode);

%% =========================
% Per-step accuracy, new method only
%% =========================
[stepNames, newBodyStepAcc, handStepAcc] = computeNewStepAccuracies( ...
    stepIndex, instruction, captureMask, ...
    expectedHandClass, newHandClass, ...
    expectedBodyClass, newBodyClass);

figure('Color','w', 'Name', 'Current Method Per-Step Accuracy');
bar([handStepAcc(:), newBodyStepAcc(:)]);
grid on; box on;

xticks(1:numel(stepNames));
xticklabels(stepNames);
xtickangle(45);
ylabel('Accuracy on capture samples (%)');
title('Per-Step Accuracy, Current Method');
legend({'Hand classifier', 'New body prototype'}, 'Location','best');

%% =========================================================
% Local functions
%% =========================================================

function tf = hasColumn(T, colName)
    tf = any(string(T.Properties.VariableNames) == string(colName));
end

%% =========================
function s = getStringColumnOrDefault(T, colName, defaultValue)
    colName = string(colName);
    if hasColumn(T, colName)
        s = string(T.(colName));
    else
        s = repmat(string(defaultValue), height(T), 1);
    end
end

%% =========================
function stepIndex = buildStepIndexFromInstructions(instruction)
    instruction = string(instruction(:));
    changeIdx = [1; find(instruction(2:end) ~= instruction(1:end-1)) + 1];
    stepIndex = zeros(numel(instruction),1);

    for k = 1:numel(changeIdx)
        idxStart = changeIdx(k);
        if k < numel(changeIdx)
            idxEnd = changeIdx(k+1) - 1;
        else
            idxEnd = numel(instruction);
        end
        stepIndex(idxStart:idxEnd) = k;
    end
end

%% =========================
function expectedHandClass = buildExpectedHandClassCumulative(expectedMuscle, expectedState)

    N = numel(expectedMuscle);
    expectedHandClass = strings(N,1);

    currentHandState = "HAND_OPEN";

    for i = 1:N
        m = lower(strtrim(string(expectedMuscle(i))));
        s = lower(strtrim(string(expectedState(i))));

        if m == "hand"
            if s == "closed" || s == "close" || s == "high"
                currentHandState = "HAND_CLOSED";
            elseif s == "open" || s == "low"
                currentHandState = "HAND_OPEN";
            end
        end

        expectedHandClass(i) = currentHandState;
    end
end

%% =========================
function expectedBodyClass = buildExpectedBodyClassCumulative(expectedMuscle, expectedState)

    N = numel(expectedMuscle);
    expectedBodyClass = strings(N,1);

    currentBodyState = "BODY_REST";

    for i = 1:N
        m = lower(strtrim(string(expectedMuscle(i))));
        s = lower(strtrim(string(expectedState(i))));

        % Default: keep previous body state.
        % This is essential during hand commands.
        if m == "biceps"
            if s == "plus" || s == "high" || s == "close" || s == "closed"
                currentBodyState = "ARM_CLOSE";
            elseif s == "off" || s == "relax" || s == "low"
                currentBodyState = "BODY_REST";
            end

        elseif m == "triceps"
            if s == "minus" || s == "extend" || s == "high" || s == "open"
                currentBodyState = "ARM_EXTEND";
            elseif s == "off" || s == "relax" || s == "low"
                currentBodyState = "BODY_REST";
            end

        elseif m == "both_arm"
            if s == "off" || s == "relax"
                currentBodyState = "BODY_REST";
            end

        elseif m == "deltoid"
            if s == "up" || s == "raise"
                currentBodyState = "HEIGHT_UP";
            elseif s == "down" || s == "drop" || s == "lower"
                currentBodyState = "HEIGHT_DOWN";
            elseif s == "off" || s == "relax" || s == "low"
                currentBodyState = "BODY_REST";
            end

        elseif m == "chest"
            if s == "plus" || s == "active" || s == "activate" || s == "high"
                currentBodyState = "CHEST_ACTIVE";
            elseif s == "off" || s == "relax" || s == "low"
                currentBodyState = "BODY_REST";
            end

        elseif m == "all"
            if s == "relax" || s == "off"
                currentBodyState = "BODY_REST";
            end

        elseif m == "hand"
            % Do not change body state.
            % Hand is independent and can happen while body state remains active.
            currentBodyState = currentBodyState;

        else
            % Unknown instruction: keep previous body state.
            currentBodyState = currentBodyState;
        end

        expectedBodyClass(i) = currentBodyState;
    end
end

%% =========================
function pct = buildIndependentPercentages(T, channels, features)

    for c = 1:numel(channels)
        ch = channels(c);
        predCol = "pred_" + ch;

        if hasColumn(T, predCol)
            currentPct = double(T.(predCol));
        else
            E = zeros(height(T),1);
            validCount = 0;

            for f = 1:numel(features)
                col = features(f) + "_" + ch;
                if hasColumn(T, col)
                    x = double(T.(col));
                    E = E + normalize01(x);
                    validCount = validCount + 1;
                end
            end

            if validCount == 0
                currentPct = zeros(height(T),1);
            else
                E = E / validCount;
                currentPct = 100 * normalize01(E);
            end
        end

        pct.(ch) = currentPct(:);
    end
end

%% =========================
function handClass = classifyHandIndependent(handPct)

    handPct = double(handPct(:));
    N = numel(handPct);

    ON  = 70;
    OFF = 40;

    handClass = strings(N,1);
    currentState = "HAND_OPEN";

    for i = 1:N
        if handPct(i) > ON
            currentState = "HAND_CLOSED";
        elseif handPct(i) < OFF
            currentState = "HAND_OPEN";
        end

        handClass(i) = currentState;
    end
end

%% =========================
function held = holdUnknownStates(rawClass, initialState)

    rawClass = string(rawClass(:));
    held = strings(size(rawClass));

    currentState = string(initialState);

    for i = 1:numel(rawClass)
        if rawClass(i) ~= "UNKNOWN" && rawClass(i) ~= "IGNORE"
            currentState = rawClass(i);
        end
        held(i) = currentState;
    end
end

%% =========================
function [X, featureNames] = buildNewFeatureMatrix(T, channels, historyLen)

    if nargin < 3 || isempty(historyLen)
        historyLen = 7;
    end

    N = height(T);
    base = [];
    featureNames = strings(0,1);

    % Absolute RMS/MAV/WL features, matching the Python body base vector.
    for c = 1:numel(channels)
        ch = channels(c);

        for feat = ["rms", "mav", "wl"]
            col = feat + "_" + ch;
            x = getCol(T, col);

            base = [base, x(:)]; %#ok<AGROW>
            featureNames(end+1,1) = col; %#ok<AGROW>
        end
    end

    % Relative body-channel energy, also matching the Python implementation:
    % E_c = max(RMS,0) + max(MAV,0) + max(WL,0)
    E = zeros(N, numel(channels));

    for c = 1:numel(channels)
        ch = channels(c);

        rms = max(0, getCol(T, "rms_" + ch));
        mav = max(0, getCol(T, "mav_" + ch));
        wl  = max(0, getCol(T, "wl_"  + ch));

        E(:,c) = rms + mav + wl;
    end

    totalE = sum(E,2) + 1e-9;
    relE = E ./ totalE;

    for c = 1:numel(channels)
        base = [base, relE(:,c)]; %#ok<AGROW>
        featureNames(end+1,1) = "rel_energy_" + channels(c); %#ok<AGROW>
    end

    % Short-term slope. Python uses the current base minus the previous base.
    slopeFeat = [zeros(1,size(base,2)); diff(base)];

    % Short-term variance over the recent body-feature history.
    varFeat = movvar(base, historyLen, 0, 1, 'Endpoints','shrink');

    X = [base, slopeFeat, varFeat];
    X(~isfinite(X)) = 0;

    originalNames = featureNames;

    for j = 1:numel(originalNames)
        featureNames(end+1,1) = "slope_" + originalNames(j); %#ok<AGROW>
    end

    for j = 1:numel(originalNames)
        featureNames(end+1,1) = "var_" + originalNames(j); %#ok<AGROW>
    end
end
%% =========================
function x = getCol(T, colName)
    if hasColumn(T, colName)
        x = double(T.(colName));
    else
        x = zeros(height(T),1);
    end
end

%% =========================
function y = normalize01(x)
    x = double(x(:));

    lo = prctile(x, 5);
    hi = prctile(x, 95);

    if abs(hi - lo) < 1e-12
        y = zeros(size(x));
    else
        y = (x - lo) / (hi - lo);
        y = max(0, min(1, y));
    end
end

%% =========================
function model = trainPrototypeClassifier(X, labels, trainMask, minStd)

    if nargin < 4 || isempty(minStd)
        minStd = 0.20;
    end

    labels = string(labels(:));
    trainMask = logical(trainMask(:));

    classes = unique(labels(trainMask));
    classes(classes == "UNKNOWN") = [];
    classes(classes == "IGNORE") = [];

    if isempty(classes)
        error('No valid classes found for training. Check expected_muscle/expected_state and test_phase columns.');
    end

    Xtrain = X(trainMask,:);
    globalMu = mean(Xtrain, 1, 'omitnan');
    globalSigma = std(Xtrain, 0, 1, 'omitnan');
    globalSigma(globalSigma < 1e-9) = 1;

    Xn = (X - globalMu) ./ globalSigma;
    Xn(~isfinite(Xn)) = 0;

    model.classes = classes;
    model.globalMu = globalMu;
    model.globalSigma = globalSigma;
    model.mu = cell(numel(classes),1);
    model.sigma = cell(numel(classes),1);

    for k = 1:numel(classes)
        cls = classes(k);
        idx = trainMask & labels == cls;

        Xc = Xn(idx,:);

        if isempty(Xc)
            model.mu{k} = zeros(1, size(X,2));
            model.sigma{k} = ones(1, size(X,2));
            continue;
        end

        model.mu{k} = mean(Xc, 1, 'omitnan');

        s = std(Xc, 0, 1, 'omitnan');
        s(s < minStd) = minStd;

        model.sigma{k} = s;
    end
end
%% =========================
function [predClass, confidence, D, classes, minMargin] = classifyPrototype(X, model, minMargin, excludedClasses)

    if nargin < 3 || isempty(minMargin)
        minMargin = 0.8;
    end
    if nargin < 4 || isempty(excludedClasses)
        excludedClasses = strings(0,1);
    end

    excludedClasses = string(excludedClasses(:));

    allClasses = string(model.classes(:));
    keep = ~ismember(allClasses, excludedClasses);
    classes = allClasses(keep);

    muCells = model.mu(keep);
    sigmaCells = model.sigma(keep);

    N = size(X,1);
    K = numel(classes);

    if K == 0
        predClass = repmat("UNKNOWN", N, 1);
        confidence = zeros(N,1);
        D = zeros(N,0);
        return;
    end

    globalSigma = model.globalSigma;
    globalSigma(globalSigma < 1e-9) = 1;

    Xn = (X - model.globalMu) ./ globalSigma;
    Xn(~isfinite(Xn)) = 0;

    D = zeros(N,K);

    for k = 1:K
        mu = muCells{k};
        sg = sigmaCells{k};

        sg(sg < 0.20) = 0.20;
        Z = (Xn - mu) ./ sg;
        D(:,k) = sqrt(sum(Z.^2, 2));
    end

    [sortedD, sortedIdx] = sort(D, 2, 'ascend');

    bestIdx = sortedIdx(:,1);
    bestD = sortedD(:,1); %#ok<NASGU>

    if K >= 2
        secondD = sortedD(:,2);
        confidence = secondD - sortedD(:,1);
    else
        confidence = zeros(N,1);
    end

    predClass = strings(N,1);

    for i = 1:N
        predClass(i) = classes(bestIdx(i));
    end

    predClass(confidence < minMargin) = "UNKNOWN";
end
%% =========================

%% =========================
function isDown = detectDeltoidZeroArmDown(T, t, windowSec, ratioThreshold, epsValue)

    rms = getCol(T, "rms_deltoid");
    mav = getCol(T, "mav_deltoid");
    wl  = getCol(T, "wl_deltoid");

    zeroFeature = (rms <= epsValue) | (mav <= epsValue) | (wl <= epsValue);

    N = height(T);
    isDown = false(N,1);

    for i = 1:N
        idx = t >= (t(i) - windowSec) & t <= t(i);
        if ~any(idx)
            isDown(i) = false;
        else
            isDown(i) = mean(zeroFeature(idx)) > ratioThreshold;
        end
    end
end

%% =========================
function opClass = bodyActionToOperationTimeline(t, bodyAction, chestReleaseConfirmSec)

    bodyAction = string(bodyAction(:));
    N = numel(bodyAction);
    opClass = repmat("BODY_REST", N, 1);

    chestHighActive = false;
    chestCurrentDirection = "CHEST_PLUS";
    chestNextDirection = "CHEST_PLUS";
    chestReleaseCandidateSince = nan;

    for i = 1:N
        action = bodyAction(i);

        if action == "CHEST_ACTIVE"
            if ~chestHighActive
                chestHighActive = true;
                chestCurrentDirection = chestNextDirection;
                chestReleaseCandidateSince = nan;
            end
            opClass(i) = chestCurrentDirection;

        else
            % If chest is no longer active, the rotation command stops
            % immediately, but the stored chest direction is not allowed to
            % change until the release has lasted for the full confirmation
            % time. This is what protects the toggle from short deactivation
            % spikes.
            if chestHighActive
                if isnan(chestReleaseCandidateSince)
                    chestReleaseCandidateSince = t(i);
                end

                if (t(i) - chestReleaseCandidateSince) >= chestReleaseConfirmSec
                    chestHighActive = false;
                    chestNextDirection = toggleChestDirection(chestCurrentDirection);
                    chestReleaseCandidateSince = nan;
                end
            else
                chestReleaseCandidateSince = nan;
            end

            if action == "ARM_CLOSE"
                % Current robot operation: biceps activation gives -Distance.
                opClass(i) = "DISTANCE_MINUS";

            elseif action == "ARM_EXTEND"
                % Current robot operation: triceps activation gives +Distance.
                opClass(i) = "DISTANCE_PLUS";

            elseif action == "HEIGHT_UP"
                opClass(i) = "HEIGHT_UP";

            elseif action == "HEIGHT_DOWN"
                opClass(i) = "HEIGHT_DOWN";

            else
                opClass(i) = "BODY_REST";
            end
        end
    end
end
%% =========================
function nextDirection = toggleChestDirection(currentDirection)

    if string(currentDirection) == "CHEST_PLUS"
        nextDirection = "CHEST_MINUS";
    else
        nextDirection = "CHEST_PLUS";
    end
end

function smoothed = temporalConfirmClasses(rawClass, confirmSamples, initialState)

    rawClass = string(rawClass(:));
    smoothed = strings(size(rawClass));

    currentAccepted = string(initialState);
    candidate = string(initialState);
    count = 0;

    for i = 1:numel(rawClass)
        r = rawClass(i);

        if r == "UNKNOWN" || r == "IGNORE"
            smoothed(i) = currentAccepted;
            continue;
        end

        if r == candidate
            count = count + 1;
        else
            candidate = r;
            count = 1;
        end

        if count >= confirmSamples
            currentAccepted = candidate;
        end

        smoothed(i) = currentAccepted;
    end
end

%% =========================
function acc = safeAccuracy(predicted, expected)

    predicted = string(predicted(:));
    expected = string(expected(:));

    if isempty(expected)
        acc = NaN;
        return;
    end

    acc = 100 * mean(predicted == expected, 'omitnan');
end

%% =========================
function level = handClassToLevel(cls)

    cls = string(cls(:));
    level = zeros(size(cls));

    for i = 1:numel(cls)
        switch cls(i)
            case "HAND_OPEN"
                level(i) = 0;
            case "HAND_CLOSED"
                level(i) = 1;
            otherwise
                level(i) = 0;
        end
    end
end

%% =========================
function level = bodyClassToLevel(cls)

    cls = string(cls(:));
    level = zeros(size(cls));

    for i = 1:numel(cls)
        switch cls(i)
            case "UNKNOWN"
                level(i) = 0;
            case "BODY_REST"
                level(i) = 1;
            case "ARM_CLOSE"
                level(i) = 2;
            case "ARM_EXTEND"
                level(i) = 3;
            case "HEIGHT_UP"
                level(i) = 4;
            case "HEIGHT_DOWN"
                level(i) = 5;
            case "CHEST_ACTIVE"
                level(i) = 6;
            otherwise
                level(i) = 0;
        end
    end
end



%% =========================
function level = bodyOperationToLevel(cls)

    cls = string(cls(:));
    level = zeros(size(cls));

    for i = 1:numel(cls)
        switch cls(i)
            case "BODY_REST"
                level(i) = 1;
            case "DISTANCE_MINUS"
                level(i) = 2;
            case "DISTANCE_PLUS"
                level(i) = 3;
            case "HEIGHT_UP"
                level(i) = 4;
            case "HEIGHT_DOWN"
                level(i) = 5;
            case "CHEST_PLUS"
                level(i) = 6;
            case "CHEST_MINUS"
                level(i) = 7;
            otherwise
                level(i) = 0;
        end
    end
end

%% =========================
function [responseIdx, delayStats] = computeClassResponseIndicators(t, instruction, expectedClass, predictedClass)

    instruction = string(instruction(:));
    expectedClass = string(expectedClass(:));
    predictedClass = string(predictedClass(:));

    changeIdx = [1; find(instruction(2:end) ~= instruction(1:end-1)) + 1];
    responseIdx = nan(numel(changeIdx), 1);
    delays = nan(numel(changeIdx), 1);

    for k = 1:numel(changeIdx)
        idx0 = changeIdx(k);
        target = expectedClass(idx0);

        if target == "UNKNOWN" || target == "IGNORE"
            continue;
        end

        if k < numel(changeIdx)
            idxLast = changeIdx(k+1) - 1;
        else
            idxLast = numel(t);
        end

        idxLocal = find(predictedClass(idx0:idxLast) == target, 1, 'first');

        if ~isempty(idxLocal)
            idxResp = idx0 + idxLocal - 1;
            responseIdx(k) = idxResp;
            delays(k) = t(idxResp) - t(idx0);
        end
    end

    validDelays = delays(~isnan(delays));
    if isempty(validDelays)
        delayStats.avgDelay = NaN;
        delayStats.maxDelay = NaN;
    else
        delayStats.avgDelay = mean(validDelays);
        delayStats.maxDelay = max(validDelays);
    end

    delayStats.stepDelay = delays;
end

%% =========================
function drawClassResponseArrows(t, instruction, responseIdx, yLimits, labelPrefix, yFraction, arrowColor)

    instruction = string(instruction(:));

    if isempty(instruction)
        return;
    end

    if nargin < 6 || isempty(yFraction)
        yFraction = 0.10;
    end

    if nargin < 7 || isempty(arrowColor)
        arrowColor = [0.10 0.10 0.10];
    end

    changeIdx = [1; find(instruction(2:end) ~= instruction(1:end-1)) + 1];

    % Fixed vertical height for this arrow group.
    % This avoids arrows jumping between different y-levels.
    yArrow = yLimits(1) + yFraction * (yLimits(2) - yLimits(1));

    % Fixed-size arrow-tip markers. These do not scale with delay duration.
    arrowMarkerSize = 2;
    arrowLineWidth  = 2.0;

    for k = 1:numel(changeIdx)
        idx0 = changeIdx(k);

        if k > numel(responseIdx) || isnan(responseIdx(k)) || responseIdx(k) <= idx0
            continue;
        end

        x0 = t(idx0);
        x1 = t(responseIdx(k));
        delaySec = x1 - x0;

        % Thick horizontal delay line.
        plot([x0 x1], [yArrow yArrow], '-', ...
            'Color', arrowColor, ...
            'LineWidth', arrowLineWidth, ...
            'HandleVisibility', 'off');

        % Small fixed-size arrow tips at both ends.
        plot(x0, yArrow, '<', ...
            'Color', arrowColor, ...
            'MarkerFaceColor', arrowColor, ...
            'MarkerSize', arrowMarkerSize, ...
            'LineWidth', arrowLineWidth, ...
            'HandleVisibility', 'off');

        plot(x1, yArrow, '>', ...
            'Color', arrowColor, ...
            'MarkerFaceColor', arrowColor, ...
            'MarkerSize', arrowMarkerSize, ...
            'LineWidth', arrowLineWidth, ...
            'HandleVisibility', 'off');

        text((x0+x1)/2, yArrow - 0.02 * (yLimits(2)-yLimits(1)), ...
            labelPrefix + sprintf('%.2fs', delaySec), ...
            'HorizontalAlignment', 'center', ...
            'VerticalAlignment', 'top', ...
            'FontSize', 11, ...
            'Color', arrowColor, ...
            'BackgroundColor', [1 1 1 0.6], ...
            'Margin', 0.5, ...
            'Interpreter', 'none');
    end
end

%% =========================
function shadeFromLogical(t, active, yLimits, shadeColor)

    active = logical(active(:));

    if ~any(active)
        return;
    end

    d = diff([false; active; false]);
    starts = find(d == 1);
    ends   = find(d == -1) - 1;

    for k = 1:length(starts)
        x1 = t(starts(k));
        x2 = t(ends(k));

        patch([x1 x2 x2 x1], ...
              [yLimits(1) yLimits(1) yLimits(2) yLimits(2)], ...
              shadeColor, ...
              'FaceAlpha', 0.10, ...
              'EdgeColor', 'none', ...
              'HandleVisibility', 'off');
    end
end

%% =========================
function drawInstructionLines(t, instruction, yLimits)

    instruction = string(instruction(:));

    if isempty(instruction)
        return;
    end

    changeIdx = [1; find(instruction(2:end) ~= instruction(1:end-1)) + 1];

    yText = yLimits(2) - 0.04 * (yLimits(2) - yLimits(1));

    for k = 1:numel(changeIdx)
        idx = changeIdx(k);
        x = t(idx);

        xline(x, '--k', 'LineWidth', 1.0, 'HandleVisibility','off');

        text(x, yText, instruction(idx), ...
            'Rotation', 90, ...
            'VerticalAlignment', 'top', ...
            'HorizontalAlignment', 'right', ...
            'FontSize', 10, ...
            'Color', 'k', ...
            'BackgroundColor', [1 1 1 0.6], ...
            'Interpreter', 'none');
    end
end


%% =========================
function drawInstructionLinesNoLabels(t, instruction, yLimits)

    instruction = string(instruction(:));

    if isempty(instruction)
        return;
    end

    changeIdx = [1; find(instruction(2:end) ~= instruction(1:end-1)) + 1];

    for k = 1:numel(changeIdx)
        idx = changeIdx(k);
        x = t(idx);

        xline(x, '--k', 'LineWidth', 1.0, 'HandleVisibility','off');
    end
end

%% =========================
function printConfusion(expected, predicted, titleText)

    expected = string(expected(:));
    predicted = string(predicted(:));

    valid = expected ~= "UNKNOWN";
    expected = expected(valid);
    predicted = predicted(valid);

    if isempty(expected)
        fprintf('\n===== %s Confusion Matrix =====\n', titleText);
        fprintf('No valid expected labels.\n');
        return;
    end

    classes = unique([expected; predicted]);

    fprintf('\n===== %s Confusion Matrix =====\n', titleText);
    fprintf('%18s', '');

    for j = 1:numel(classes)
        fprintf('%18s', classes(j));
    end

    fprintf('\n');

    for i = 1:numel(classes)
        fprintf('%18s', classes(i));

        for j = 1:numel(classes)
            fprintf('%18d', sum(expected == classes(i) & predicted == classes(j)));
        end

        fprintf('\n');
    end
end

%% =========================
function Dn = normalizeDistanceColumns(D)

    D = double(D);
    Dn = zeros(size(D));

    for j = 1:size(D,2)
        x = D(:,j);
        lo = min(x, [], 'omitnan');
        hi = max(x, [], 'omitnan');

        if ~isfinite(lo) || ~isfinite(hi) || abs(hi - lo) < 1e-12
            Dn(:,j) = zeros(size(x));
        else
            Dn(:,j) = (x - lo) / (hi - lo);
        end
    end

    Dn(~isfinite(Dn)) = 0;
end

%% =========================
function stateColors = getStateColorsFromDistancePlots(classNames, pDist)

    defaultColors = lines(5);
    stateColors.BODY_REST    = defaultColors(1,:);
    stateColors.ARM_CLOSE    = defaultColors(2,:);
    stateColors.ARM_EXTEND   = defaultColors(3,:);
    stateColors.HEIGHT_UP    = defaultColors(4,:);
    stateColors.HEIGHT_DOWN  = defaultColors(4,:);
    stateColors.CHEST_ACTIVE = defaultColors(5,:);

    classNames = string(classNames(:));

    for k = 1:numel(classNames)
        cls = classNames(k);
        if k <= numel(pDist) && isgraphics(pDist(k))
            switch cls
                case "BODY_REST"
                    stateColors.BODY_REST = pDist(k).Color;
                case "ARM_CLOSE"
                    stateColors.ARM_CLOSE = pDist(k).Color;
                case "ARM_EXTEND"
                    stateColors.ARM_EXTEND = pDist(k).Color;
                case "HEIGHT_UP"
                    stateColors.HEIGHT_UP = pDist(k).Color;
                case "HEIGHT_DOWN"
                    stateColors.HEIGHT_DOWN = pDist(k).Color;
                case "CHEST_ACTIVE"
                    stateColors.CHEST_ACTIVE = pDist(k).Color;
            end
        end
    end

    % If HEIGHT_DOWN is not a separate trained class, use HEIGHT_UP color.
    if ~any(classNames == "HEIGHT_DOWN")
        stateColors.HEIGHT_DOWN = stateColors.HEIGHT_UP;
    end
end

%% =========================
function [Y, names] = buildRawSignalMatrix(T, channels)

    Y = zeros(height(T), numel(channels));
    names = strings(numel(channels),1);

    for c = 1:numel(channels)
        [Y(:,c), colUsed] = getRawChannelSignal(T, channels(c));
        names(c) = upperFirst(channels(c)) + " (" + colUsed + ")";
    end
end

%% =========================
function [x, colUsed] = getRawChannelSignal(T, ch)

    ch = string(ch);

    candidates = [ ...
        "raw_" + ch, ...
        ch + "_raw", ...
        "emg_" + ch, ...
        ch + "_emg", ...
        "adc_" + ch, ...
        ch + "_adc", ...
        "analog_" + ch, ...
        ch + "_analog", ...
        "sample_" + ch, ...
        ch + "_sample", ...
        ch];

    for k = 1:numel(candidates)
        if hasColumn(T, candidates(k)) && isnumeric(T.(candidates(k)))
            x = double(T.(candidates(k)));
            colUsed = candidates(k);
            return;
        end
    end

    % Fallback: if raw columns were not logged, use the least-processed
    % available signal so the plot still works.
    fallbackCandidates = ["wl_" + ch, "rms_" + ch, "mav_" + ch, "pred_" + ch];

    for k = 1:numel(fallbackCandidates)
        if hasColumn(T, fallbackCandidates(k)) && isnumeric(T.(fallbackCandidates(k)))
            warning('No raw column found for %s. Plotting %s instead.', ch, fallbackCandidates(k));
            x = double(T.(fallbackCandidates(k)));
            colUsed = fallbackCandidates(k) + " fallback";
            return;
        end
    end

    available = strjoin(string(T.Properties.VariableNames), ', ');
    error('No raw or fallback signal column found for channel "%s". Available columns are:\n%s', ch, available);
end

%% =========================
function relEnergy = computeRelativeEnergy(T, channels)

    N = height(T);
    E = zeros(N, numel(channels));

    for c = 1:numel(channels)
        ch = channels(c);

        rms = max(0, getCol(T, "rms_" + ch));
        mav = max(0, getCol(T, "mav_" + ch));
        wl  = max(0, getCol(T, "wl_"  + ch));

        E(:,c) = rms + mav + wl;
    end

    totalE = sum(E, 2) + 1e-9;

    relEnergy = E ./ totalE;
    relEnergy(~isfinite(relEnergy)) = 0;
end
%% =========================
function out = upperFirstArray(s)

    s = string(s(:));
    out = strings(size(s));

    for k = 1:numel(s)
        out(k) = upperFirst(s(k));
    end
end

%% =========================
function out = upperFirst(s)

    s = char(string(s));

    if isempty(s)
        out = string(s);
    else
        out = string([upper(s(1)) s(2:end)]);
    end
end


%% =========================
function [stepNames, newBodyStepAcc, handStepAcc] = computeNewStepAccuracies( ...
    stepIndex, instruction, captureMask, ...
    expectedHandClass, predHandClass, ...
    expectedBodyClass, newBodyClass)

    steps = unique(stepIndex);
    stepNames = strings(numel(steps),1);

    newBodyStepAcc = nan(numel(steps),1);
    handStepAcc    = nan(numel(steps),1);

    for k = 1:numel(steps)
        idxStep = stepIndex == steps(k);
        firstIdx = find(idxStep, 1, 'first');

        if isempty(firstIdx)
            stepNames(k) = "step " + string(steps(k));
        else
            stepNames(k) = instruction(firstIdx);
        end

        idxHand = idxStep & captureMask & expectedHandClass ~= "UNKNOWN";
        idxBody = idxStep & captureMask & expectedBodyClass ~= "UNKNOWN";

        if any(idxHand)
            handStepAcc(k) = safeAccuracy(predHandClass(idxHand), expectedHandClass(idxHand));
        end

        if any(idxBody)
            newBodyStepAcc(k) = safeAccuracy(newBodyClass(idxBody), expectedBodyClass(idxBody));
        end
    end
end
%% =========================
function drawnDelays = extractDrawnResponseDelays(t, instruction, responseIdx)

    instruction = string(instruction(:));
    responseIdx = responseIdx(:);

    changeIdx = [1; find(instruction(2:end) ~= instruction(1:end-1)) + 1];

    drawnDelays = [];

    for k = 1:numel(changeIdx)
        idx0 = changeIdx(k);

        % This exactly matches the condition used by drawClassResponseArrows:
        % it only draws an arrow if the response index exists and is after idx0.
        if k > numel(responseIdx) || isnan(responseIdx(k)) || responseIdx(k) <= idx0
            continue;
        end

        idxResp = responseIdx(k);
        drawnDelays(end+1,1) = t(idxResp) - t(idx0); %#ok<AGROW>
    end
end