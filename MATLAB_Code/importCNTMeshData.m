% Imports the data from the CNT Mesh output csv
mainResultsFolder = 'C:/Users/Gabory/Dropbox/Research/OutputFiles/';
folder_name = uigetdir(mainResultsFolder);
filesList = dir([folder_name '\*.csv']);


for fileNum=1:2;%length(filesList)
    %Get the current filename
    fileName = filesList(fileNum).name;
    filePath = [folder_name '\' fileName];
    
    %CNT Num
    cnt.num = str2num(cell2mat(regexp(fileName,'\d+','match'))); %#ok<ST2NM>
    %Open the file to get non-position data
    currFile = fopen(filePath,'r');
    %Chirality
    line = strcat(fgets(currFile));
    temp = strsplit(line,{';' ',' ' '});
    cnt.chirality.n = temp(2);
    cnt.chirality.m = temp(3);
    clear temp;
    %Length
    line = strcat(fgets(currFile));
    temp = strsplit(line,{';' ',' ' '});
    cnt.length = str2double(temp(2));
    clear temp;
    %Cylinder Height
    line = strcat(fgets(currFile));
    temp = strsplit(line,{';' ','});
    cnt.cylHeight = str2double(temp(2));
    clear temp;
    %Tube separation
    line = strcat(fgets(currFile));
    temp = strsplit(line,{';' ','});
    cnt.tubeSeparation = str2double(temp(2));
    clear temp;
    %Cylinder spacing
    line = strcat(fgets(currFile));
    temp = strsplit(line,{';' ','});
    cnt.cylSeparation = str2double(temp(2));
    clear temp;
    fclose(currFile);
    
    %Get the position data
    [cnt.x, cnt.y, cnt.z] = importPositionData(filePath);
    %cntList(fileNum) = cnt;
   
end
