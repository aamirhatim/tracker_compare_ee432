function tracker()
files = dir('data');

for i = 3:length(files)
    path = "data/"+files(i).name+"/";
    
    boosting = csvread(path+"BOOSTING.csv");
    kcf = csvread(path+"KCF.csv");
    medianflow = csvread(path+"MEDIANFLOW.csv");
    mil = csvread(path+"MIL.csv");
    tld = csvread(path+"TLD.csv");
    
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
    hold off;
    
end

end