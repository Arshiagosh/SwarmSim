% PackForAI.m
% This script reads all .m files in the current folder and subfolders
% and combines them into a single text file to send to an AI.

clc; clear;
outputFileName = 'AI_Codebase.txt';

% Get the names of the packing/unpacking scripts to exclude them
thisFile = mfilename('fullpath');
[~, thisFileName, ~] = fileparts(thisFile);
unpackFileName = 'UnpackFromAI';

% Get all .m files recursively (Requires MATLAB R2016b or newer)
files = dir('**/*.m');

fidOut = fopen(outputFileName, 'w');
if fidOut == -1
    error('Cannot create output file: %s', outputFileName);
end

filesAdded = 0;

for i = 1:length(files)
    % Skip the pack and unpack scripts themselves so they aren't modified
    if contains(files(i).name, thisFileName) || contains(files(i).name, unpackFileName)
        continue;
    end
    
    fullFilePath = fullfile(files(i).folder, files(i).name);
    
    % Get relative path for the AI header
    currentDir = pwd;
    relPath = erase(fullFilePath, [currentDir filesep]);
    
    % Replace backslashes with forward slashes for cross-platform AI reading
    relPath = strrep(relPath, '\', '/');
    
    % Write the BEGIN marker
    fprintf(fidOut, '<--- BEGIN FILE: %s --->\n', relPath);
    
    % Read file contents
    fileText = fileread(fullFilePath);
    
    % Write contents to the text file
    fprintf(fidOut, '%s', fileText);
    
    % Ensure it ends with a newline before writing the END marker
    if ~isempty(fileText) && fileText(end) ~= newline && fileText(end) ~= char(13)
        fprintf(fidOut, '\n');
    end
    
    % Write the END marker
    fprintf(fidOut, '<--- END FILE: %s --->\n\n', relPath);
    filesAdded = filesAdded + 1;
end

fclose(fidOut);
fprintf('Successfully packed %d files into %s\n', filesAdded, outputFileName);
