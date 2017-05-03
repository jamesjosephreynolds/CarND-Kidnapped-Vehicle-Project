%%
%
% Load data files (run m scripts)
%
p_data
gt_data
l_data

%% 
%
% Create movie object
%
try
  aviobj = avifile('example.avi');
catch err
  aviobj = close(aviobj);
  aviobj = avifile('example.avi');
end

%%
%
% Make movie from individual plots
%
for i = 1:length(G)
  % Sort particle data and get relative particle size
  P_sort = [];
  cnt = 1;
  P_temp = sortrows(P{i+1},[1,2]);
  for j = 1:length(P{i+1})
      
      if j == 1
          P_sort(cnt,1) = P_temp(j,1);
          P_sort(cnt,2) = P_temp(j,2);
          P_sort(cnt,3) = 1;
          cnt = cnt + 1;
      elseif (P_sort(cnt-1,1) == P_temp(j,1)) &&...
              (P_sort(cnt-1,2) == P_temp(j,2))
          P_sort(cnt-1,3) = P_sort(cnt-1,3) + 1;
      else
          P_sort(cnt,1) = P_temp(j,1);
          P_sort(cnt,2) = P_temp(j,2);
          P_sort(cnt,3) = 1;
          cnt = cnt + 1;
      end
  end
    
  % UI information
  fprintf(['Image #',int2str(i),'\n'])
  
  fig_handle = figure('visible','on');
  % plot landmarks
  subplot(2,1,1)
  scatter(L(:,1), L(:,2),40,'dk','filled');
  hold('on');
  %plot ground truths
  scatter(G{i}(1), G{i}(2),500,'pr','filled');
  title(['Landmarks and ground truth position']);
  hold('off');
  
  subplot(2,1,2)
  % plot particles
  for j = 1:length(P_sort)
    scatter(P_sort(j,1), P_sort(j,2),10*P_sort(j,3),'ob','filled');
    hold('on')
  end
  
  % plot ground truths
  scatter(G{i}(1), G{i}(2),500,'pr','filled');
  % scale axes
  axis([G{i}(1)-1, G{i}(1)+1, G{i}(2)-0.5, G{i}(2)+0.5]);
  title(['Ground truth position and particles at timestep ',int2str(i)]);
  hold('off');
  
  % Convert fig to frame
  frame_handle = getframe(fig_handle);
  close(fig_handle);
  % Convert frame to image for manipulation
  image_handle = frame2im(frame_handle);
  % Make image smaller
  small_handle = imresize(image_handle, [240,320]);
  small_frame_handle = im2frame(small_handle);
  % Update movie object
  aviobj = addframe(aviobj, small_frame_handle);
end

aviobj = close(aviobj);