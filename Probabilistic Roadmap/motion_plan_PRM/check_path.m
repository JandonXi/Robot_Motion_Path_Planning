function feasibility = check_path(point_current,point_end,map)
feasibility=true;
step=max(abs(point_current(1)-point_end(1)),abs(point_current(2)-point_end(2)));
path_x=linspace(point_current(2),point_end(2),step+1);
path_y=linspace(point_current(1),point_end(1),step+1);
for i=1:step+1
    if ~check_point([round(path_y(i)),round(path_x(i))],map)
        feasibility=false;
    end
end
end

