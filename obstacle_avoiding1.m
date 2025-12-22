clc; clear;

fieldw = 100; fieldh = 100;
nStatic = 8; nDynamic = 5; nCirc = nStatic + nDynamic;
circStatic = [rand(nStatic,1)*fieldw, rand(nStatic,1)*fieldh, ones(nStatic,1)*3];
circDynamic = [rand(nDynamic,1)*fieldw, rand(nDynamic,1)*fieldh, ones(nDynamic,1)*2.3];
circV = (rand(nDynamic,2)-0.5)*2.0;
circObs = [circStatic; circDynamic];

% Interactive: Get start/goal locations
figure('Color','w');
axis([0 fieldw 0 fieldh]); grid on; xlabel('X'); ylabel('Y');
title('Click ONCE for start, ONCE for goal...');
[xs, ys] = ginput(1); start = [xs ys];
[xg, yg] = ginput(1); goal  = [xg yg];
close;

robotPos = start;
robotPath = robotPos;
endEffTrace = robotPos;
repRange = 13;

% --- Speed/step control slider window ---
sliderFig = figure('Name','Robot Step Size','NumberTitle','off', 'Position',[100 100 220 80]);
sliderStep = uicontrol('Parent',sliderFig,'Style','slider','Position',[45 30 120 20],...
    'min',0.3,'max',2,'value',0.9,'TooltipString','Robot Step Size');
label = uicontrol('Parent',sliderFig,'Style','text','Position',[50 10 110 15],...
    'String','Robot Step Size');

% --- Main animation window ---
robotFig = figure('Name','Robot Path Planning','NumberTitle','off','Color','w');
set(robotFig, 'Position', [400 100 700 700]);

dangerSoundFlag = false;

for iter = 1:800
    robotStep = get(sliderStep,'Value');   % Live step from slider

    circDynamic(:,1:2) = circDynamic(:,1:2) + circV;
    for j=1:nDynamic
        if circDynamic(j,1)-circDynamic(j,3)<0 || circDynamic(j,1)+circDynamic(j,3)>fieldw, circV(j,1) = -circV(j,1); end
        if circDynamic(j,2)-circDynamic(j,3)<0 || circDynamic(j,2)+circDynamic(j,3)>fieldh, circV(j,2) = -circV(j,2); end
    end
    circObs = [circStatic; circDynamic];

    % --- Potential Field Planning ---
    k_att = 1.5;
    v_att = k_att * (goal - robotPos) / (norm(goal - robotPos) + 1e-6);
    k_rep = 40;
    v_rep = [0 0];
    minDist = 1e6;
    for i=1:nCirc
        d = norm(robotPos - circObs(i,1:2)) - circObs(i,3);
        if d < minDist, minDist = d; end
        if d > 0 && d < repRange
            dir = (robotPos - circObs(i,1:2)) / (norm(robotPos - circObs(i,1:2)) + 1e-6);
            v_rep = v_rep + k_rep * (1/d - 1/repRange) * dir / (d^2 + 1e-6);
        end
    end
    v_tot = v_att + v_rep + 0.08*randn(1,2);

    safeStep = robotStep;
    for tryStep = linspace(robotStep, 0.3, 4)
        moveVec = tryStep * v_tot / norm(v_tot + 1e-8);
        candPos = robotPos + moveVec;
        hit = false;
        nCheckPoints = 13;
        buffer = tryStep + 1.2;
        for s = linspace(0, 1, nCheckPoints)
            probe = robotPos + moveVec*s;
            for i=1:nCirc
                if norm(probe - circObs(i,1:2)) < circObs(i,3) + buffer
                    hit = true; break;
                end
            end
            if hit, break; end
        end
        if ~hit
            safeStep = tryStep;
            v_tot = moveVec;
            break;
        end
    end

    robotPos = robotPos + v_tot;
    robotPath = [robotPath; robotPos];
    endEffTrace = [endEffTrace; robotPos];
    distToGoal = norm(robotPos - goal);

    % --- Visualization ---
    figure(robotFig); cla; hold on;
    % Fade obstacle color based on robot proximity (addon #3)
    for i=1:nCirc
        dist = norm(robotPos-circObs(i,1:2));
        c = 1 - min(dist/repRange,1); % fade: closer = more red
        viscircles(circObs(i,1:2), circObs(i,3)+1.2, 'Color',[c 0.3 0.7],'LineStyle','--');
        % Danger zone
        viscircles(circObs(i,1:2), repRange, 'Color',[1 0.8 0],'LineStyle',':','LineWidth',1.2);
    end
    % Path trace
    plot(endEffTrace(:,1), endEffTrace(:,2),'c--','LineWidth',1.5);
    % Path
    plot(robotPath(:,1),robotPath(:,2),'b-','LineWidth',2.0);
    % Robot arrow
    if norm(v_tot)>0
        head = v_tot/norm(v_tot)*2;
        quiver(robotPos(1), robotPos(2), head(1), head(2), 0, 'k', 'MaxHeadSize', 2, 'LineWidth',2);
    end
    % Start/goal
    plot(start(1),start(2),'bo','MarkerFaceColor','b','MarkerSize',10);
    plot(goal(1),goal(2),'gp','MarkerFaceColor','g','MarkerSize',18);
    text(fieldw-20, fieldh-4, sprintf('Step: %d', iter), 'FontSize',13, 'Color','k', 'BackgroundColor',[1 1 1 0.3]);
    title(sprintf('Robot Path Planning | Goal Dist: %.2f', distToGoal));
    xlim([0 fieldw]); ylim([0 fieldh]); axis equal tight;
    grid on; box on;

    % Sound alert if near collision (addon #4)
    if minDist < 2.5 && ~dangerSoundFlag
        sound(sin(1:400)*700,7000); dangerSoundFlag=true;
    end
    if minDist > 2.5, dangerSoundFlag=false; end

    drawnow;
    pause(0.008);

    if distToGoal < 2, break; end
end

fprintf('Final path length: %.2f\n', sum(sqrt(sum(diff(endEffTrace).^2,2))) );
fprintf('Steps taken: %d\n', size(robotPath,1));
