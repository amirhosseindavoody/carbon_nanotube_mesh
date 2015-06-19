function [ cntList ] = importCNTMeshData( resultsFolderPath )
%importCNTMeshData Imports all data from CNT Mesh .csv output files
%  
folder_name = ' ';
if nargin == 1
    folder_name = resultsFolderPath; %pre-selected folder
else
    folder_name = uigetdir(pwd); %folder dialog

end

