function yaml_map = readYaml(yaml_filename)
    
yaml_map = containers.Map;

% Open the file
fid = fopen(yaml_filename);

% wait for start
tline = fgetl(fid);
temp_str = split(tline,':');
yaml_map(temp_str{1}) = strtrim(temp_str{2});

while(~feof(fid))
    tline = fgetl(fid);
    temp_str = split(tline,':');
    yaml_map(temp_str{1}) = strtrim(temp_str{2});
end
