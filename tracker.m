function tracker()
files = dir('data');
fps = zeros(length(files)-2, 5);

for i = 3:length(files)
    path = "data/"+files(i).name+"/";
    
    boosting = csvread(path+"BOOSTING.csv");
    kcf = csvread(path+"KCF.csv");
    medianflow = csvread(path+"MEDIANFLOW.csv");
    mil = csvread(path+"MIL.csv");
    tld = csvread(path+"TLD.csv");
        
    %% Get average FPS for each tracker
    avg = mean(boosting(:,4));
    fps(i-2,1) = avg;
    avg = mean(kcf(:,4));
    fps(i-2,2) = avg;
    avg = mean(medianflow(:,4));
    fps(i-2,3) = avg;
    avg = mean(mil(:,4));
    fps(i-2,4) = avg;
    avg = mean(tld(:,4));
    fps(i-2,5) = avg;
    
    
    %% Plot position of bounding box
    figure('Name', files(i).name)
    subplot(2,1,1)
    hold on;
    plot(1:length(boosting), boosting(:,3))
    plot(1:length(kcf), kcf(:,3))
    plot(1:length(medianflow), medianflow(:,3))
    plot(1:length(mil), mil(:,3))
    plot(1:length(tld), tld(:,3))
    legend("Boosting", "KCF", "MedianFlow", "MIL", "TLD")
    title("Y Position over Time");
    ylabel("Position (px)");
    xlabel("Time (ms)");
    ylim([-5 480]);
    hold off;
    
    subplot(2,1,2)
    hold on;
    plot(1:length(boosting), boosting(:,2))
    plot(1:length(kcf), kcf(:,2))
    plot(1:length(medianflow), medianflow(:,2))
    plot(1:length(mil), mil(:,2))
    plot(1:length(tld), tld(:,2))
    legend("Boosting", "KCF", "MedianFlow", "MIL", "TLD")
    title("X Position over Time");
    ylabel("Position (px)");
    xlabel("Time (ms)");
    ylim([-5 640]);
    hold off;
    
end

%% Get overal FPS average
fps_mean = mean(fps);
labels = ["Boosting" "KCF" "MedianFlow" "MIL" "TLD"];
figure('Name', 'Average FPS')
bar(fps_mean)
set(gca, 'xticklabels', labels)
title("Average FPS");
ylabel("Frames per Second");
xlabel("Algorithm");

end