% UnpackFromAI.m
% This script reads the updated text file from the AI and overwrites
% your local .m files with the new code.

clc; clear;
inputFileName = 'AI_Codebase.txt';

if ~isfile(inputFileName)
    error('Input file %s does not exist. Please place it in this directory.', inputFileName);
end

fidIn = fopen(inputFileName, 'r');
if fidIn == -1
    error('Cannot open input file.');
end

currentFileId = -1;
currentRelPath = '';
filesUpdated = 0;

while ~feof(fidIn)
    % fgets keeps the newline character, preserving exact formatting
    line = fgets(fidIn); 
    
    % Check if the line is a BEGIN marker
    tokens = regexp(line, '^<--- BEGIN FILE: (.*?) --->', 'tokens');
    if ~isempty(tokens)
        currentRelPath = tokens{1}{1};
        % Convert AI's forward slashes back to your OS's file separators
        currentRelPath = strrep(currentRelPath, '/', filesep);
        
        % Ensure directory exists (if file was inside a subfolder)
        [fileDir, ~, ~] = fileparts(currentRelPath);
        if ~isempty(fileDir) && ~exist(fileDir, 'dir')
            mkdir(fileDir);
        end
        
        % Open the specific .m file for writing (overwrites old file)
        currentFileId = fopen(currentRelPath, 'w');
        if currentFileId == -1
            warning('Could not write to file: %s', currentRelPath);
        else
            fprintf('Updating: %s\n', currentRelPath);
            filesUpdated = filesUpdated + 1;
        end
        continue;
    end
    
    % Check if the line is an END marker
    tokensEnd = regexp(line, '^<--- END FILE: (.*?) --->', 'tokens');
    if ~isempty(tokensEnd)
        if currentFileId ~= -1
            fclose(currentFileId);
            currentFileId = -1;
        end
        continue;
    end
    
    % If a file is currently "open", write the code line to it
    if currentFileId ~= -1
        fprintf(currentFileId, '%s', line);
    end
end

fclose(fidIn);
fprintf('\nSuccessfully unpacked and updated %d files from %s\n', filesUpdated, inputFileName);
