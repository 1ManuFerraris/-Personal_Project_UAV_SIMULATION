%% FLIGHT DATA LOGGING SYSTEM
% Description: Exports telemetry data to a specific folder, creating it if necessary.

% 1. Initialization and User Input
folder_name = 'telemetry_data'; % Nombre de la carpeta de destino
filename_base = input('Ingrese el nombre del archivo: ', 's');
if isempty(filename_base), filename_base = 'flight_telemetry'; end

% 2. Folder Management
% Check if the folder exists; if not, create it.
if ~exist(folder_name, 'dir')
    mkdir(folder_name);
    fprintf('Folder "%s" created successfully.\n', folder_name);
end

% 3. Telemetry Structure Definition
blocks = {'Log_Nav', 'PID_Effort', 'Motor_PWM', 'Ground_Truth', 'Measurements'};

% 4. Header Definitions
headers.Log_Nav = {'Vel_X','Vel_Y','Vel_Z','Pos_X','Pos_Y','Pos_Z',...
    'Roll_rad','Pitch_rad','Yaw_rad','p_rate','q_rate','r_rate',...
    'Status_ID','Flight_Mode','System_Ready','Batt_Percent','Batt_Volt',...
    'SP_Roll','SP_Pitch','SP_Yaw','SP_Mode','SP_Arm'};

headers.PID_Effort = {'Throttle_u', 'Roll_u', 'Pitch_u', 'Yaw_u'};
headers.Motor_PWM = {'Motor_1', 'Motor_2', 'Motor_3', 'Motor_4'};

headers.Ground_Truth = {'Vel_X_GT', 'Vel_Y_GT', 'Vel_Z_GT', ...
    'Pos_X_GT', 'Pos_Y_GT', 'Pos_Z_GT', ...
    'Roll_GT', 'Pitch_GT', 'Yaw_GT', ...
    'p_GT', 'q_GT', 'r_GT'};

headers.Measurements = {'Vel_X_m', 'Vel_Y_m', 'Vel_Z_m', ...
    'Pos_X_m', 'Pos_Y_m', 'Pos_Z_m', ...
    'Roll_m', 'Pitch_m', 'Yaw_m', ...
    'p_m', 'q_m', 'r_m'};

% 5. Execution Loop
fprintf('Starting Telemetry Export to /%s...\n', folder_name);

for i = 1:length(blocks)
    block_name = blocks{i};
    
    try
        raw_data = out.(block_name);
        t = raw_data.time;
        values = squeeze(raw_data.signals.values);
        
        if size(values, 2) == length(t), values = values'; end
        
        T = table(t, 'VariableNames', {'Time_s'});
        current_headers = headers.(block_name);
        [~, num_vars] = size(values);
        
        for j = 1:num_vars
            if j <= length(current_headers)
                T.(current_headers{j}) = values(:, j);
            else
                T.(sprintf('%s_ext_%d', block_name, j)) = values(:, j);
            end
        end
        
        % Construct Path: folder/filename_block.csv
        full_output_path = fullfile(folder_name, sprintf('%s_%s.csv', filename_base, block_name));
        
        writetable(T, full_output_path);
        fprintf('[SUCCESS] Exported: %s\n', full_output_path);
        
    catch ME
        fprintf('[WARNING] Failed to process block: %s. Error: %s\n', block_name, ME.message);
    end
end

fprintf('Export Process Finished.\n');