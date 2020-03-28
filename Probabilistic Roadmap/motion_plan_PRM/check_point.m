function feasibility = check_point( point , map )
if map(point(1),point(2))==0
    feasibility=false;
else
    feasibility=true;
end
end

