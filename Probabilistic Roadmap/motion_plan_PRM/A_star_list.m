function A_list = A_star_list( vertex,self_index,goal_index,parent_index )
h_cost=straight_distance(vertex(self_index,:),vertex(2,:)); % heuristic_cost
g_cost=straight_distance(vertex(self_index,:),vertex(goal_index,:));
f_cost=h_cost+g_cost;
A_list=[self_index,parent_index,g_cost,f_cost];
end

