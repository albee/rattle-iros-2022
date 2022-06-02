function plot_features(loc_features, t_traj_start)
  feats = [];
  t_feats = [];
  for i = 1:1:length(loc_features)
     t_feats(i) = stamp2time(loc_features{i}.Header.Stamp) - t_traj_start;
     feats(i) = length(loc_features{i}.FeatureArray);
  end
  plot(t_feats, feats);
  title('Feature Counts')
  ylabel('Features [-]')
  xlabel('Time [s]')
  end