% Put the directory lisiting here where Rosbag Files are stored
% This file reads rosbag file that has topic named /DistanceEstimator/dist
% and convert them to .mat file
% Developer: Rahul Kumar Bhadani
% rahulbhadani@email.arizona.edu

DIRPath = './';
Files=dir(DIRPath);
cd(DIRPath);
for k=1:length(Files)
   FileNames=Files(k).name;

    if ((strcmp(FileNames,'.') == 0 || strcmp(FileNames,'..') == 0 ) && length(FileNames) > 3)
        if strcmp(FileNames(end-2:end),'bag')
            bag = rosbag(FileNames);
            bagselect1 = select(bag, 'Topic','/DistanceEstimator/dist');
            msgs  = readMessages(bagselect1);
            Distance = zeros(1,length(msgs));
            for i = 1:length(msgs)
                Distance(i) = msgs{i}.Data;
            end
            ts = timeseries(bagselect1, 'Data');
            matFile = FileNames(1:end-4);
            matFile = strcat(matFile,'.mat');
            save(matFile,'ts','Distance');
        end
    end
end