% Function to convert time from '#HH:MM:SS.MMM' format to seconds
function time_in_seconds = convert_time_to_seconds(time_str)
    % Remove the leading '#' character
    time_str = strrep(time_str, '#', '');
    
    % Split the string by ':' and '.'
    time_parts = regexp(time_str, '[:.]', 'split');
    
    % Extract hours, minutes, seconds, and milliseconds
    hours = str2double(time_parts{1});
    minutes = str2double(time_parts{2});
    seconds = str2double(time_parts{3});
    milliseconds = str2double(time_parts{4});
    
    % Convert to total seconds
    time_in_seconds = hours * 3600 + minutes * 60 + seconds + milliseconds / 1000;
end
