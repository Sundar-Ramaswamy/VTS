%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Description: A script for generating speed from integrated acceleration.
%   However, to do it the optimal offset adjusted acceleration must be found. 
%   Used in projects, but further developed to be used in the Vehicle and 
%   Traffic Safety Course (TME202) at Chalmers University of Technology, Sweden.
%
% Indata: oDAll.mat - a three field Matlab struct
%
% Output:
%       afit - a struct with the parameter information for the model
%       A set of figures
%
% Version   Date            Developer           Organisation
% 1-3                       Jonas B‰rgman       Used in a few projects and for scientific papers
% 4.0       2017-12-01      Jonas B‰rgman       Chalmers, modification for students
% 5.0       2020-11-06      Jonas B‰rgman       Chalmers, minor updates
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Load the data (you likely have to change the path)
load('C:\Users\user\Desktop\CHALMERS\TME202\AS_SignalProcessing_Matlab\AccOffset\oDAllTimeSync.mat');

clear oDResults;

% Loop thouugh all the datasets. Do the offsetadjustment for each.
for n1=1:length(oDAll)
    
    %% Prepare data and do a first check
    % This is the one to use now
    oD=oDAll(n1);
    
    % Plot data directly
    figure(101)
    clf; % Clear graph
    plot(oD.time, oD.fGPSSpeedRawKmph, 'b*')
    hold on % Make it possible to plot multiple plots on top of eachother
    plot(oD.time, oD.fXaccRawG, 'r.-')
    ylabel({'GPS speed [km/h]', 'X acceleration [g]'})
    xlabel('Time [s]')
    % This was no good. Why? Show that a lot of nans.
    oD.fGPSSpeedRawKmph
    
    % Get indices that are not zero
    iGPSSpeedRawNotIsNan=~isnan(oD.fGPSSpeedRawKmph);
    % Plot only the data without nans (connect the data points)
    plot(oD.time(iGPSSpeedRawNotIsNan), ...
        oD.fGPSSpeedRawKmph(iGPSSpeedRawNotIsNan),'b.-')
    ylabel({'GPS speed [m/s]','X acceleration [m/s2]'})
    xlabel('Time [s]')
    
    % We do not have data in SI-units
    oD.fEgoSpeed=oD.fGPSSpeedRawKmph / 3.6;
    % Again, find data points that may be nan
    oD.IEgospeedNotIsNan=find(~isnan( oD.fEgoSpeed));
    
    % This time, fill in the nans with linear interpolation
    oD.fEgoSpeedInterp=interp1(oD.time(oD.IEgospeedNotIsNan), ...
        oD.fEgoSpeed(oD.IEgospeedNotIsNan), ...
        oD.time);
    
    % Also, we have acceleration in G. Make SI-units
    oD.fXaccOrig=oD.fXaccRawG * 9.8;
    
    % Prepare a new plot
    figure(103)
    clf
    sLegend103=[];
    
    % Basics, interpolated.
    plot(oD.time, ...
        oD.fEgoSpeedInterp,'b.-')
    sLegend103{1}='Interpolated GPS-speed';
    hold on
    plot(oD.time,oD.fXaccOrig,'r.-')
    sLegend103{length(sLegend103)+1}='Original x-acceleration';

    ylabel({'Speed [m/s]','X acceleration [m/s^2]'})
    xlabel('Time [s]')
    
    % To make sure we have no small gaps,also here fill in the nans
    %   with linear interpolation
    IXaccNotIsNan=~isnan(oD.fXaccOrig); % Alternative to using "find"...
    oD.fXacc=interp1(oD.time(IXaccNotIsNan), ...
                     oD.fXaccOrig(IXaccNotIsNan), ...
                     oD.time);
    
    %% Now we have plotted and prepared the data. Now it is time to 
    %%    integrate the acceleration and get the offset for it. 
    % Now integrate the acceleration to get speed
    oD.fVintxacc=cumtrapz(oD.time, oD.fXacc);
    
    % Add to the plot
    plot(oD.time, oD.fVintxacc,'k.-')
    sLegend103{length(sLegend103)+1}='Integrated original acceleration';
    
    % We have not "offset it", just integrated the acceleration. Will start at 0.
    %  Choose a good point, relatively stable, manually.
    fStablePointForOffset=ginput();
    
    % Find the index of this point. Get the index of the first time
    %   time is greater than the picked time
    IStablePointForOffset=find(oD.time >= fStablePointForOffset(1),1,'First');
    
    % Offset the intergrated acceleration at that point
    %   Do it by getting the difference between the GPS-speed
    %   and integrated acceleration at that point
    fVOffsetCorr=oD.fEgoSpeedInterp(IStablePointForOffset) ...
        - oD.fVintxacc(IStablePointForOffset);
 
    % Now, create the offset adjusted integrated acceleration
    oD.fVintxaccOff= oD.fVintxacc + fVOffsetCorr;
    
    % Add to the plot
    plot(oD.time, oD.fVintxaccOff, 'g.-')
    sLegend103{length(sLegend103)+1}='Integrated speed-offset acceleration';

    %% Now you see that the integrated acceleration is not quite good. We 
    %   need to find what the xacc offset is that produce the best possible 
    %   fit with the GPS speed. Loop to identify optimal xacc offset
    % [Extra (own) assignment off-line: can this be optimized by not checking all
    % values? try to implement some other way of doing this. ]
    
    % Choose a set to of parameters for the optimization
    fAccOffsetSet = 0.01;
    fStartEndOffset = 1.5;

    % Now, there is a clear offset, lets loop over small increments. That
    % is, create a vector with these increments. 
    fAccOffsetVector= -fStartEndOffset:fAccOffsetSet:fStartEndOffset; % [m/s2]
    
    % Clear the cost variable, as it is new for each iteration
    clear fCostIt
    bLegendAdded=0;
    
    %% Loop over the vector and calculate 
    for n2=1:length(fAccOffsetVector)
        % Add the offset to acceleration for this iteration
        fXaccOffset=oD.fXacc + fAccOffsetVector(n2);
        
        % Integrate the acceleration to get speed
        fVintxaccOffset=cumtrapz(oD.time, fXaccOffset);
        
        % Offset the intergrated acceleration at the point of offsetting
        % (from before)
        fVOffsetCorr=oD.fEgoSpeedInterp(IStablePointForOffset) ...
            - fVintxaccOffset(IStablePointForOffset);
        
        fVintXaccOffset= fVintxaccOffset + fVOffsetCorr;
        
        %% Calculate a "cost"/error function between the speed and the GPS
        fDiffIntAndGPSSpeed=fVintXaccOffset - oD.fEgoSpeedInterp;
        
        % Here, the cost function is simple. Use just a sum of all
        % absolute errors. Many more sophisticated methods can be used...
        % Note that sum does not work with nans, therefor we only sum the
        % non-nans.
        % [Extra assignment: is this a good cost function? Should some
        % other function be used? Try different?]
        fCostIt(n2)=sum(abs(fDiffIntAndGPSSpeed(~isnan(fDiffIntAndGPSSpeed))));
        
        % Plot every 10th (remainder of the loop variable = 0)
        if rem(n2,10)==0
            plot(oD.time, fVintXaccOffset, 'k--', 'LineWidth', 0.5)
            
            
            
            
            plot(oD.time, fXaccOffset, 'm--', 'LineWidth', 0.5)

            if ~bLegendAdded
                sLegend103{length(sLegend103)+1}='Integrated xacc with xacc-offset';
                sLegend103{length(sLegend103)+1}='Xacc with xacc-offset';

                % For legend preparation only. That is, add some dummy plot
                % to simplify creating legends later
                plot(0, 0, 'c.-', 'LineWidth',2)
                sLegend103{length(sLegend103)+1}='Final integrated acceleration with best xacc-offset';
                bLegendAdded=1;
            end
            % Pause for 0.5s between plots
            pause(0.5)            
        end        
    end
    
    %% Find the index of the best acceleration offset, ploy it
    [~,IMinCost] = min(fCostIt);
    figure(104)
    clf
    plot(fCostIt,'b.-')
    hold on
    plot(IMinCost, fCostIt(IMinCost),'rx','MarkerSize',14,'LineWidth',3);
    
    %Add this offset to the data and in the struct
    oD.fXaccCorr=oD.fXacc + fAccOffsetVector(IMinCost);
    % Integrate agin
    oD.fVintxaccOffset=cumtrapz(oD.time, oD.fXaccCorr);
    % Again, calculate the speed offset
    fVOffsetCorr=oD.fEgoSpeedInterp(IStablePointForOffset) ...
        - oD.fVintxaccOffset(IStablePointForOffset);
    
    %% Create the integrated acceleration vector with the offset correction 
    % and add to the integrated speed
    oD.fVintXaccCorr= oD.fVintxaccOffset + fVOffsetCorr;
    
    ylabel('Cost function value');
    xlabel('Cost vector index');
    
    figure(103)
    % Add to the plot. This is the "new speed".
    plot(oD.time, oD.fVintXaccCorr, 'c.-', 'LineWidth',2)
    ylabel('Speed [m/s]');
    xlabel('Time [s]');
    legend(sLegend103,'Location','SouthWest')
    
    %% Check if the integrated speed for the GPS and the speed is similar
    % Integrate speeds to get distance
    figure(105)
    clf
    fDistInAcc=cumtrapz(oD.time, oD.fVintXaccCorr);
    fDistGPS=cumtrapz(oD.time, oD.fEgoSpeedInterp);
    
    % Plot it, but offset it to the point chosen for the speed offset
    plot(oD.time - oD.time(IStablePointForOffset), ...
         fDistInAcc - fDistInAcc(IStablePointForOffset), ...
         'b-', 'LineWidth', 3);
     hold on
    plot(oD.time - oD.time(IStablePointForOffset), ...
         fDistGPS - fDistGPS(IStablePointForOffset), ...
         'r-', 'LineWidth', 3);

    % Add informatin for the legend in a dynamic way     
    sLegend{1}='Distance from integrated integrated acc';

    hold on;
    
    % Same as above, reset the zero but for GPS speed
    plot(oD.time - oD.time(IStablePointForOffset), ...
         fDistGPS - fDistGPS(IStablePointForOffset), ...
         'r', 'LineWidth', 3);
    % Add informatin for the legend in a dynamic way     
    sLegend{length(sLegend) + 1}='Distance from integrated GPS speed';

    % Add informatin for the legend in a dynamic way
    sLegend{length(sLegend) + 1}='Point of speed offset';
     
    
    % Show the legend
    legend(sLegend,'Location','SouthEast')
   
    ylabel('Distance traveled, zero at crash [m]')
    xlabel('Time [s]')
    
    oDResults(n1).oD = oD;
end
save('D:\1_Work\4_Utbildning\VTS2018\Matlab÷vning\AccOffset\oDResultsAccOffset.mat','oDResults')