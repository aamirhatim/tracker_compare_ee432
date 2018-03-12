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
    hold on;
    plot(boosting(:,2), boosting(:,3))
    plot(kcf(:,2), kcf(:,3))
    plot(medianflow(:,2), medianflow(:,3))
    plot(mil(:,2), mil(:,3))
    plot(tld(:,2), tld(:,3))
    hold off;
end






end