% RTTemperatureFit
clear temperatureCalDataMat;

% read in data from saved file
temperatureFile = fopen('temperatureRaw.dta', 'r');
temperatureCalDataMat = fscanf(temperatureFile, '%f %f %f %f %f %f %f %f %f %f', [10, inf]);
fprintf('Mat size = %d\n', size(temperatureCalDataMat));
fclose(temperatureFile);

% open output file
corrFile = fopen('temperatureCorr.dta', 'w');

% sort the data in ascending order
[s,si]=sort(temperatureCalDataMat(10,:));
temperatureCalDataMat = temperatureCalDataMat(:,si);

for i = 1:9 
    % Conduct Polynomial Fit
    s = temperatureCalDataMat(i,:);
    t = temperatureCalDataMat(10,:);
    p = polyfit( t, s, 3 ); % the polynom
    sf= polyval( p, t);     % the fitted data
    % Debug
    % sfh=p(4) + p(3)*t + p(2)*t.*t + p(1)*t.*t.*t;
    % figure(i); clf; plot(t,s,'o'); hold on; plot(t,sf); plot(t,sfh, 'k');
    %
    % Identify out layers outside average +/- 2 * std
    residual = (s-sf);
    err_est=std(residual);
    avg_est=mean(residual);
    ok = find(residual < avg_est + 2 * err_est & residual > avg_est - 2 * err_est);
    % Debug
    % figure(i+10); clf; plot(t,residual); hold on; plot(t(outlayer),residual(outlayer),'ro') plot(t,avg_est*(ones(size(t)))); plot(t,(avg_est+2*err_est)*(ones(size(t))), 'g'); plot(t,(avg_est-2*err_est)*(ones(size(t))), 'g');
    t=t(ok);
    s=s(ok);
    p = polyfit( t, s, 3 );
    sf= polyval( p, t);
    % Debug
    % figure(i); clf; plot(t,s,'o'); hold on; plot(t,sf);
    %
    % Remove offset of data at 32.5 deg C
    % We are only interested in temperature related changes and not actual values of accel,gyro,compass
    if ((32.5 > max(t)) || (32.5 < min(t)))
        t_center =  mean(t);
    else
        t_center = 32.5;
    end
    bg= polyval( p, t_center);
    s=s-bg;
    p = polyfit( t, s, 3 );
    sf= polyval( p, t);
    rsquared = sum((s-sf).^2 / std(s-sf).^2) / length(s);
    % Debug
    figure(i); clf; plot(t,s,'o'); hold on; plot(t,sf);
    c0(i)=p(4); % const
    c1(i)=p(3); % t^1
    c2(i)=p(2); % t^2
    c3(i)=p(1); % t^3
    switch i
        case 1
            print('-djpeg', 'Accelx.tiff'); 
        case 2
            print('-djpeg', 'Accely.tiff'); 
        case 3
            print('-djpeg', 'Accelz.tiff'); 
        case 4
            print('-djpeg', 'Gyrox.tiff'); 
        case 5
            print('-djpeg', 'Gyroy.tiff'); 
        case 6
            print('-djpeg', 'Gyroz.tiff'); 
        case 7
            print('-djpeg', 'Compassx.tiff'); 
        case 8
            print('-djpeg', 'Compassy.tiff'); 
        case 9
            print('-djpeg', 'Compassz.tiff'); 
    end
end

close all

cMat = [c0;c1;c2;c3] ; 

for j=1:4
   for i=1:9
      fprintf(corrFile, '%f ', cMat(j,i));
   end
   fprintf(corrFile, '\n');
end	
	
fclose(corrFile);

