list_bags = dir('C:\Users\Admin\Desktop\New_Tests_New_Wings');
threshold_std_pitch = 8;
threshold_std_altitude = 0.08;
mkdir frequency_altitude
for kk = 1:length(list_bags)
    W = contains(num2str(list_bags(kk).name),'bag');
    if W
        filename = [num2str(list_bags(kk).name)];
    num = regexp(filename,'[\d\.]+','match');
    num = str2double(num);
    mkdir(sprintf('Objective Altitude= %dcm',num(1)));
    filePath = fullfile(cd,filename);
    bag = rosbag(filePath);
    % Select the topics
    topics = bag.AvailableTopics;
    bjoy = select(bag,'Topic','/joy');
    msgStructsjoy = readMessages(bjoy,'DataFormat','struct');
    bSel = select(bag,'Topic','/vrpn_client_node/bionic/pose');
    xPoints = cellfun(@(m) double(m.Axes(4)),msgStructsjoy,'UniformOutput',false);
    tail_command = cell2mat(xPoints);
    t_tail_command = bjoy.MessageList.Time;
    joy = timeseries(tail_command,t_tail_command);

    % tail control
    orientation_X = timeseries(bSel, 'Pose.Orientation.X');
    orientation_Y = timeseries(bSel, 'Pose.Orientation.Y');
    orientation_Z = timeseries(bSel, 'Pose.Orientation.Z');
    orientation_W = timeseries(bSel, 'Pose.Orientation.W');
    position_X = timeseries(bSel, 'Pose.Position.X');
    position_Y = timeseries(bSel, 'Pose.Position.Y');
    position_Z = timeseries(bSel, 'Pose.Position.Z');

    %Time
    t_vc = orientation_X.Time;
    ti = t_vc(1);
    for i = 1:length(t_vc)
        t_vc(i) = t_vc(i) - ti;
    end

    %Positionstopics
    posX_vc = position_X.Data;
    posY_vc = position_Y.Data;
    posZ_vc = position_Z.Data;
    
    %Orientations roll pitch yaw
    orientation = [orientation_W.data orientation_X.data orientation_Y.data orientation_Z.data];
    orientation_euler = quat2eul(orientation); %yaw pitch roll
    pitch_vc = rad2deg(orientation_euler(:,2));
    roll_vc = rad2deg(orientation_euler(:,3));
    yaw_vc = rad2deg(orientation_euler(:,1));

    %% Detect the different rounds
    [~,initcross_y,finalcross_y,~,midlev_y] = dutycycle(posY_vc,t_vc,'StateLevels',[-0.05 0.05]);
    [~,initcross_x,finalcross_x,~,midlev_x] = dutycycle(posX_vc,t_vc,'StateLevels',[-0.05 0.05]);
    initcross = initcross_x;
    finalcross = 2*initcross(length(initcross_x)) - initcross(length(initcross_x)-1);
    t_rounds = {};
    pitch_rounds_vc = {};
    pitch_rounds = {};
    altitude_rounds = {};
    for i=1:length(initcross)
        time_round = [];
        pitch_round = [];
        altitude_round = [];
        for j=1:length(t_vc)
            if i < length(initcross) && t_vc(j) >= initcross(i) && t_vc(j) <= initcross(i+1)
                time_round = [time_round, t_vc(j)];
                altitude_round = [altitude_round, posZ_vc(j)];
                pitch_round = [pitch_round, pitch_vc(j)];
                t_rounds(i) = {time_round};
                altitude_rounds(i) = {altitude_round};
                pitch_rounds_vc(i) = {pitch_round};
            elseif i == length(initcross) && t_vc(j) >= initcross(i) && t_vc(j) <= finalcross
                time_round = [time_round, t_vc(j)];
                altitude_round = [altitude_round, posZ_vc(j)];
                pitch_round = [pitch_round, pitch_vc(j)];
                t_rounds(i) = {time_round};
                altitude_rounds(i) = {altitude_round};
                pitch_rounds_vc(i) = {pitch_round};
            end
        end
    end

    %% Calculate the flapping frequency for each round
    Fs = 500; % Sampling frequency                 
    T = 1/Fs; % Sampling period
    f_min = 10; % min frequency
    f_max = 20; % max frequency 
    f_rounds = {};
    flapping_frequency = {};
    fft_pitch_rounds = {};
    for i=1:length(initcross)
        [pitch_vc_ro, tf] = filloutliers(pitch_rounds_vc{i},'nearest');
        L = length(pitch_rounds_vc{i}); % Length of signal
        t = (0:L-1)*T;
        fft_pitch_vc = fft(pitch_vc_ro);
        P2 = abs(fft_pitch_vc/L);
        P1 = P2(1:L/2+1);
        P1(2:end-1) = 2*P1(2:end-1);
        f = Fs*(0:(L/2))/L;
        f_rounds(i) = {Fs*(0:(L/2))/L};

        % Filtre passe haut
        wn=2*f_min/Fs;
        [b,a] = butter(9,wn,'high');
        freqz(b,a);
        pitch_vc_filtered = filter(b,a,pitch_vc_ro);

        % Filtre passe bas après application du passe haut
        wn2 = 2*f_max/Fs;
        [b2,a2] = butter(9,wn2,'low');
        freqz(b2,a2);
        pitch_vc_low_filtered = filter(b2,a2,pitch_vc_filtered);

        % Spectre filtré par le passe bas après filtrage par le passe haut
        fft_pitch_vc_low_filtered = fft(pitch_vc_low_filtered);
        P2_low_filtered = abs(fft_pitch_vc_low_filtered/L);
        P1_low_filtered = P2_low_filtered(1:L/2+1);
        P1_low_filtered(2:end-1) = 2*P1_low_filtered(2:end-1);
        fft_pitch_rounds(i) = {P1_low_filtered};
        pitch_rounds(i) = {pitch_vc_low_filtered};
        for j = 1:length(f)
            if P1_low_filtered(j) == max(P1_low_filtered)
                flapping_frequency(i) = {f(j)};
            end
        end
    end

    %% Create structures
    rounds = {};
    f_z = [];
    for i=1:length(initcross)
        rounds(i).time = t_rounds{i};
        rounds(i).altitudes = altitude_rounds{i};
        rounds(i).Pitch = pitch_rounds_vc{i};
        rounds(i).stdPitch = std(pitch_rounds_vc{i});
        rounds(i).meanAltitude = mean(altitude_rounds{i});
        rounds(i).stdAltitude = std(altitude_rounds{i});
        rounds(i).FlappingFrequency = flapping_frequency{i};
        f_z = [f_z; mean(altitude_rounds{i}) flapping_frequency{i}];
    end
    rounds(1).stdFrequency = std([rounds.FlappingFrequency]);
    rounds(1).meanFlappingFrequency = mean([rounds.FlappingFrequency]);
    var_rounds = strcat('N_round= ',num2str(length(initcross)),', Altitude= ',num2str(mean([rounds.meanAltitude])),'m, Frequency = ', num2str(mean([rounds.FlappingFrequency])), 'Hz.mat');
    var_rounds_fig = strcat('N_round= ',num2str(length(initcross)),', Altitude= ',num2str(mean([rounds.meanAltitude])),'m, Frequency = ', num2str(mean([rounds.FlappingFrequency])), 'Hz.fig');
    var_rounds_png = strcat('N_round= ',num2str(length(initcross)),', Altitude= ',num2str(mean([rounds.meanAltitude])),'m, Frequency = ', num2str(mean([rounds.FlappingFrequency])), 'Hz.png');
    cd(sprintf('Objective Altitude= %dcm',num(1)))
    save(var_rounds, 'rounds')
    cd C:\Users\Admin\Desktop\New_Tests_New_Wings
    variableCreator(sprintf('f_z_%i', num(1)),f_z)
    cd(sprintf('Objective Altitude= %dcm',num(1)))
    save(sprintf ('Workspace_test_%icm.mat', num(1)))


    % create latex file
    name_latex_file = strcat('N_round= ',num2str(length(initcross)),', Altitude= ',num2str(mean([rounds.meanAltitude])),'m, Frequency = ', num2str(mean([rounds.FlappingFrequency])), 'Hz.tex');
    fid = fopen(name_latex_file, 'wt' );
    fprintf( fid, '\\documentclass[aspectratio=169]{beamer}\n');
    fprintf( fid, '\\usepackage{url}\n');
    fprintf( fid, '\\usepackage{xcolor}\n');
    fprintf( fid, '\\usepackage{listings}\n');
    fprintf( fid, '\\usepackage{multicol}\n');
    fprintf( fid, '\\usepackage{xparse}\n');
    fprintf( fid, '\\usepackage{subcaption}\n');
    fprintf( fid, '\\NewDocumentCommand{\codeword}{v}{\n');
    fprintf( fid, '\\texttt{\\textcolor{blue}{#1}}}\n');
    fprintf( fid, '\\begin{document}\n');
    fprintf( fid, '\\begin{frame}{}\n');
    fprintf( fid, '\\begin{itemize}\n');
    fprintf( fid, '\\Huge{\\item Objective altitude: $%dcm$}\n', num(1));
    fprintf( fid, '\\item \\Huge{Number of rounds: $%d$}\n', length(initcross));
    fprintf( fid, '\\end{itemize}\n');
    fprintf( fid, '\\end{frame}\n');
    fprintf( fid, '\\begin{frame}{\\Huge{Summary: $%d$ rounds - $%d$cm}}\n',length(initcross), num(1));
   
    %% plot figures
    H = figure;
    A = [rounds.meanAltitude];
    B = [rounds.FlappingFrequency];
    A_best = [];
    B_best = [];
    Nselected_rounds = 0;
    for i=1:length(initcross)
        if std(altitude_rounds{i}) < threshold_std_altitude && std(pitch_rounds_vc{i}) < threshold_std_pitch
            Nselected_rounds = Nselected_rounds + 1;
            scatter(A(i), B(i),'x');hold on;
            text(A(i),B(i),strcat(num2str(i)));
            hold on;
            grid on
            grid minor
            title('Flapping-Frequency = f(Altitude)');
            xlabel('Altitude in m') 
            ylabel('Flapping-frequency in Hz')
            A_best = [A_best, A(i)];
            B_best = [B_best, B(i)];
        end
    end
    k = boundary(A_best', B_best', 0.1);
    hold on;
    plot(A_best(k), B_best(k));
    saveas(H, sprintf('FlappingFrequency-Altitude_round_Test_%dcm.png',num(1)));
    cd C:\Users\Admin\Desktop\New_Tests_New_Wings\frequency_altitude
    savefig(H, var_rounds_fig)
    close(H);
    cd C:\Users\Admin\Desktop\New_Tests_New_Wings
    cd(sprintf('Objective Altitude= %dcm',num(1)))
    fprintf( fid, '\\begin{itemize}\n');
    fprintf( fid, '\\item Number of the selected rounds: $%d$\n', Nselected_rounds);
    fprintf( fid, '\\item Mean altitude: $%.3fm$\n', mean([rounds.meanAltitude]));
    fprintf( fid, '\\item Mean flapping frequency: $%.3fHz$\n', rounds(1).meanFlappingFrequency);
    fprintf( fid, '\\item Standard deviation of the flapping frequency: $%.3fHz$\n', rounds(1).stdFrequency);
    fprintf( fid, '\\end{itemize}\n');
    fprintf( fid, '\\begin{figure}\n');
    fprintf( fid, '\\centering\n');
    fprintf( fid, '\\begin{subfigure}[b]{0.45\\textwidth}\n');
    fprintf( fid, '\\centering\n');
    fprintf( fid, '\\includegraphics[width=\\textwidth]{Trajectory_Test_%d.png}\n', num(1));
    fprintf( fid, '\\caption{2D Trajectory}\n');
    fprintf( fid, '\\end{subfigure}\n');
    fprintf( fid, '\\hfill\n');
    fprintf( fid, '\\begin{subfigure}[b]{0.45\\textwidth}\n');
    fprintf( fid, '\\centering\n');
    fprintf( fid, '\\includegraphics[width=\\textwidth]{Height_Test_%d.png}\n', num(1));
    fprintf( fid, '\\caption{Altitude during the flight.}\n');
    fprintf( fid, '\\end{subfigure}\n');
    fprintf( fid, '\\end{figure}\n');
    fprintf( fid, '\\end{frame}\n')

    for i=1:length(initcross)
        if std(altitude_rounds{i}) < threshold_std_altitude && std(pitch_rounds_vc{i}) < threshold_std_pitch
            h = figure;
            plot(rounds(i).time,rounds(i).Pitch); hold on;
            grid on
            grid minor
            title('Pitch signal for one loop');
            xlabel('Time in seconds');
            ylabel('Pitch in deg');
            savefig(h, sprintf('Pitch_round_%d_Test_%d.fig',i,num(1)));
            saveas(h, sprintf('Pitch_round_%d_Test_%d.png',i,num(1)));
            close(h);
            % fft of filtered pitch
            L = length(pitch_rounds_vc{i});
            f = Fs*(0:(L/2))/L;
            hh = figure;
            plot(f,fft_pitch_rounds{i});hold on;
            scatter(B(i), max(fft_pitch_rounds{i}),'MarkerEdgeColor',[0 .5 .5],...
              'MarkerFaceColor',[0 .7 .7], 'LineWidth',1.5);
            text(B(i),max(fft_pitch_rounds{i}),strcat('(',num2str(B(i)),'Hz)')) ;
            grid on
            grid minor
            title('Single-Sided Amplitude Spectrum of the pitch signal');
            xlabel('f (Hz)');
            ylabel('|spectre amplitude(f)| filtered');
            savefig(hh, sprintf('fft_pitch_round_%d_Test_%d.fig',i,num(1)));
            saveas(hh, sprintf('fft_pitch_round_%d_Test_%d.png',i,num(1)));
            close(hh);
            fprintf( fid, '\\begin{frame}{Details - Round %d}\n', i);
            fprintf( fid, '\\begin{itemize}\n');
            fprintf( fid, '\\item Mean altitude: $%.3fm$\n', rounds(i).meanAltitude);
            fprintf( fid, '\\item Standard deviation of the altitude: $%.3fm$\n', rounds(i).stdAltitude);
            fprintf( fid, '\\item Standard deviation of the pitch: $%.3f^{\\circ}$\n', rounds(i).stdPitch);
            fprintf( fid, '\\end{itemize}\n');
            fprintf( fid, '\\begin{figure}\n');
            fprintf( fid, '\\centering\n');
            fprintf( fid, '\\begin{subfigure}[b]{0.495\\textwidth}\n');
            fprintf( fid, '\\centering\n');
            fprintf( fid, '\\includegraphics[width=\\textwidth]{Pitch_round_%d_Test_%d.png}\n', i, num(1));
            fprintf( fid, '\\caption{Pitch measured during one round.}\n');
            fprintf( fid, '\\end{subfigure}\n');
            fprintf( fid, '\\hfill\n');
            fprintf( fid, '\\begin{subfigure}[b]{0.495\\textwidth}\n');
            fprintf( fid, '\\centering\n');
            fprintf( fid, '\\includegraphics[width=\\textwidth]{fft_pitch_round_%d_Test_%d.png}\n', i, num(1));
            fprintf( fid, '\\caption{FFT of the measured pitch.}\n');
            fprintf( fid, '\\end{subfigure}\n');
            fprintf( fid, '\\end{figure}\n');
            fprintf( fid, '\\end{frame}\n');
        end
    end
    hhh = figure;
    plot(t_vc, posZ_vc); hold on;
    hold on;
    grid on
    grid minor
    title('Altitude variation');
    xlabel('Time in seconds');
    ylabel('Altitude in m');
    savefig(hhh, sprintf('Height_Test_%d.fig',num(1)));
    saveas(hhh, sprintf('Height_Test_%d.png',num(1)));
    close(hhh);

    hhhh = figure;
    plot(posX_vc, posY_vc); hold on;
    hold on;
    grid on
    grid minor
    title('Trajectory - Circle');
    xlabel('X in m');
    ylabel('Y in m');
    savefig(hhhh, sprintf('Trajectory_Test_%d.fig',num(1)));
    saveas(hhhh, sprintf('Trajectory_Test_%d.png',num(1)));
    close(hhhh);
    fprintf( fid, '\\begin{frame}{Flapping frequency = f(Altitude)}\n');
    fprintf( fid, '\\begin{center}\n');
    fprintf( fid, '\\includegraphics[width = 0.65\\paperwidth]{FlappingFrequency-Altitude_round_Test_%dcm.png}\n', num(1));
    fprintf( fid, '\\end{center}\n');
    fprintf( fid, '\\end{frame}\n');
    fprintf( fid, '\\end{document}\n');
    cd C:\Users\Admin\Desktop\New_Tests_New_Wings
    end
end
