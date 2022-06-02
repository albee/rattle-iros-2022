function track_start_idx = get_track_start_idx(mpc_msgs)
  %{
  Get the start idx of when trajectory tracking began
  %}
  track_start_idx = 0;
  for i = 1:1:length(mpc_msgs)
      if strcmp(mpc_msgs{i}.ControlMode.Data, 'track') || strcmp(mpc_msgs{i}.ControlMode.Data, 'track_tube')
          track_start_idx = i;
          break;
      end
  end
  track_start_idx = track_start_idx + 0;
end