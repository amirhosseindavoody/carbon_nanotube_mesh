% Imports the data from the CNT Mesh output csv
mainResultsFolder = 'C:/Users/Gabory/Dropbox/Research/OutputFiles/';
folder_name = uigetdir(mainResultsFolder);
filesList = dir([folder_name '\*.csv']);
cntList = cnt(length(filesList),1);


for fileNum=1:length(filesList)
    %Get the current filename
    fileName = filesList(fileNum).name;
    filePath = [folder_name '\' fileName];
    
    %CNT Num
    cntList(fileNum).num = str2num(cell2mat(regexp(fileName,'\d+','match'))); %#ok<ST2NM>
    %Open the file to get non-position data
    currFile = fopen(filePath,'r');
    %Chirality
    line = strcat(fgets(currFile));
    temp = strsplit(line,{';' ',' ' '});
    cntList(fileNum).chirality.n = temp(2);
    cntList(fileNum).chirality.m = temp(3);
    clear temp;
    %Length
    line = strcat(fgets(currFile));
    temp = strsplit(line,{';' ',' ' '});
    cntList(fileNum).length = str2double(temp(2));
    clear temp;
    %Cylinder Height
    line = strcat(fgets(currFile));
    temp = strsplit(line,{';' ','});
    cntList(fileNum).cylHeight = str2double(temp(2));
    clear temp;
    %Tube separation
    line = strcat(fgets(currFile));
    temp = strsplit(line,{';' ','});
    cntList(fileNum).tubeSeparation = str2double(temp(2));
    clear temp;
    %Cylinder spacing
    line = strcat(fgets(currFile));
    temp = strsplit(line,{';' ','});
    cntList(fileNum).cylSeparation = str2double(temp(2));
    clear temp;
    fclose(currFile);
    
    %Get the position data
    [cntList(fileNum).x, cntList(fileNum).y, cntList(fileNum).z] = ...
        importPositionData(filePath);
   
end