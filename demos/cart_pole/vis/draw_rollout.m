function draw_rollout(result, j, dt, plant_sys, control_sys, title_text)    
for r = 1:size(result.trajectory,1)-1
    draw_cart_pole(result.trajectory(r,1), result.trajectory(r,3), result.controls(r),  ...
          ['trial # ' num2str(j) ', t=' num2str(r*dt) ' sec'], ...
          [title_text], plant_sys, control_sys);
    pause(dt);
end

end