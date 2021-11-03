%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Description: A script for fitting a three piece pice-wise linear
% model to the acceleration data. Used in projects, but further developed
% to be used in the Vehicle and Traffic Safety Course (TME202) at Chalmers
% University of Technology, Sweden.
%
% Indata: oDAll.mat - a three field Matlab struct
%
% Output:
%       afit - a struct with the parameter information for the model
%       A set of figures
%
% Version   Date            Developer           Organisation
% 1-6                       Jonas Bärgman       Used in a few projects and for scientific papers
% 7.0       2017-11-01      Jonas Bärgman       Chalmers - for the Automorive Engineering Project course
% 8.0       2017-12-01      Jonas Bärgman       Chalmers, modification for students
% 9.0       2020-11-16      Jonas Bärgman       Chalmers, slight update.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% Load the data. STuDENTS: you likely need to change the path
load('C:\BoxSync\1_Work\1_Utbildning\VTS\VTS2020\ASSignalProcessing\ToShare\FitAccModel\oAccOffsetData.mat');

% Manually find the point of start of evasive maneuver by plotting speed
% and acceleration and mark manually where the evasive maneuver seems to be
figure(10)
clf
for n1=1:length(oDResults)
    
    % Simplify use
    oD=oDResults(n1).oD;
    subplot(1,length(oDResults),n1);
    plot(oD.time, oD.fXaccCorr,'r')
    hold on
    plot(oD.time, oD.fVintXaccCorr,'b')
    
    % To easier set the manual reaction point, "zoom in".
    xlim([10 16])
    
    % Get the users input
    disp('Click on the start of evasive maneuver')
    fUserInput=ginput(1);

    % Store the data in the original struct for potential later use
    oDResults(n1).oD.iDriverStartEM=find(fUserInput(1)<=oD.time,1);
    plot(fUserInput(1),fUserInput(2),'rx','MarkerSize',13,'LineWidth',2)
end
legend('Offset corrected x-acceleration', 'Speed from corrected x-acc','Manually marked start of evasive maneuver')
ylabel({'Speed [m/s]','Acceleration [m/s^2]'})
xlabel('Time [s]');


% Loop over the events, again
for n1=1:length(oDResults)
    
    % Simplify
    oD=oDResults(n1).oD;
    
    %% Setup what "space" should be searched
    % Number of samples that are used for fitting prior to the evasive maneuver start
    SamplesBeforeEMstart=10; % = 1 s
    % How many samples before the crash that the data fit should stop.
    SampleEndBeforeCrash=1; % = 0.1 s
    
    % The index of the start of the segment for which the brake profile 
    %   should be fitted. Either at the start of the data, or SamplesBeforeEMstart
    %   before the drivers evasive maneuver
    ICostStart=max(1,oD.iDriverStartEM-SamplesBeforeEMstart); 

    % The index of the end of the segment for which the brake profile
    %   should be fitted. Note that the crash point is always at 151 samples.
    ICostEnd=151-SampleEndBeforeCrash; 
    
    %% Set the search space for the data fit
    va0=-2:0.5:2;       % m/s
    vjB=-50:0.5:-0.1;   % m/s3
    vItB=110:150;       % Matlb index
    va1=-12:0.25:-1;    % m/s
    
    
    % The total number of iterations that will be made in the grip search
    iTotalIterations=length(va0)*length(vjB)*length(vItB)*length(va1);
    disp(['Total number of iterations=' num2str(iTotalIterations)]);

    % Make some more initializations
    i=1; % Just a counter
    cost=nan(1,iTotalIterations); % The cost function vector
    params=nan(iTotalIterations,4); % The actual parameters for all iterations

    % Set how often status about the model fit grid search/optimization should
    % be shown on the screen. 0.1 = every 10 % of the data.
    fPercent=0.1;

    %% Loop through the parameter space to calculate the "cost" (error) 
    %   in the fitting. For each, extract the current value. 
    for Ia0=1:length(va0)
        a0=va0(Ia0);
        for IjB=1:length(vjB)
            jB=vjB(IjB);
            for IItB=1:length(vItB)
                ItB=vItB(IItB);
                for Ia1=1:length(va1)
                    a1=va1(Ia1);
                    
                    % Generate an acceleration profile with the same
                    %   duration as the event profile, but with the
                    %   variables that build the model.
                    xacc_gen=accprofilecreate(oD.time, ...
                                              oD.fXaccCorr, ...
                                              a0, ...
                                              jB, ...
                                              ItB, ...
                                              a1);
                    
                    % Calculate the difference for each individual
                    %   point between the original (event) acceleraton
                    %   and the model acceleration
                    % [extra assignment: You can change this cost function.
                    % Is it sensitive to the choice of cost function?]
                    vcost=abs(oD.fXaccCorr(ICostStart:ICostEnd) ...
                              -xacc_gen(ICostStart:ICostEnd)');
                    
                    % The "cost" (error) is the sum of all. This could
                    % be made different, e.g. using root/square.
                    cost(i)=sum(vcost);
                    
                    % Add the parametes to the output matrix
                    params(i,1)=a0;
                    params(i,2)=jB;
                    params(i,3)=ItB;
                    params(i,4)=a1;
                    
                    % A new iteration...
                    i=i+1;
                    
                    % Just to see how far we got at the prompt...                    
                    if i>iTotalIterations*fPercent
                        disp([num2str(fPercent*100) '%']);
                        fPercent=fPercent+0.1;
                    end
                end
            end
        end
    end
    
    % The best fit model is the one with minimum "cost"
    [~,IMinCost]=min(cost);
    
    % Now create the profile with the "best" fit model
    xacc_gen=accprofilecreate(oD.time, ...
                              oD.fXaccCorr, ...
                              params(IMinCost,1), ...
                              params(IMinCost,2), ...
                              params(IMinCost,3), ...
                              params(IMinCost,4));
    
    % Create a struct with parameters as output 
    afit.a0_all(n1)=params(IMinCost,1);
    afit.jB_all(n1)=params(IMinCost,2);
    afit.ItB_all(n1)=params(IMinCost,3);
    afit.a1_all(n1)=params(IMinCost,4);
    
    % Plot it...
    figure(100 + n1);
    
    clf;
    % The data used for the fitting
    IUsed=ICostStart:ICostEnd;
    plot(oD.time,oD.fXaccCorr,'gx', ...
                             'LineWidth',2, ...
                             'MarkerSize',15);
    hold on
    % Plot the 
    plot(oD.time(IUsed),oD.fXaccCorr(IUsed),'bx', ...
                                            'LineWidth',2, ...
                                            'MarkerSize',15)
    plot(oD.time,xacc_gen,'r','LineWidth',3)

    % Set the legend
    legend({'FV xacc','FV xacc for fit','Model'})
    
    % Set the limits to "zoom in" some
    xlim([oD.time(ICostStart)-1 oD.time(ICostEnd)+1])
    ylim([-12 3])
    
    % Set lables etc
    xlabel('Event time [s]','FontSize',14);
    ylabel('FV acceleration [m/s^2]','FontSize',14)
 
        % If you want to print the figure, use this, but update path and uncomment.
    %   Uncomment...
    %print('-djpeg', '-r300',['F:\\AccModelFit_.jpg']);
    
end


