% 路径规划-PRM概率路线图方法
% 参考: 机器人路径规划-概率路线图(PRM)方法https://zhuanlan.zhihu.com/p/65673502
%       PRM路径规划算法 - XXX已失联 - 博客园https://www.cnblogs.com/21207-iHome/p/7209954.html
%       A星算法详解(个人认为最详细,最通俗易懂的一个版本)https://blog.csdn.net/hitwhylz/article/details/23089415
%% initialize
% read the map and thresholding
img_map=imread('map.bmp');
img_bw=im2bw(img_map);

% set parms 
point_start=[10,10];
point_goal=[490,490];
num_sample=200;
distance_threshold=120;

% check the start and goal point
if ~check_point(point_start,img_bw)||~check_point(point_goal,img_bw)
    error('selected point lies on an obstacle or outside of the map!');
end

% generate the sample vertex set
vertex=[point_start;point_goal];
while(size(vertex,1)<num_sample+2)
    point_sample=randi(size(img_bw,1),[1,2]);
    if check_point(point_sample,img_bw);
        vertex=[vertex;point_sample];
    end
end

% generate adjacency matrix
path_state=zeros(num_sample+2,num_sample+2);
for i = 1:num_sample+2
    for j = 1:num_sample+2
        if straight_distance(vertex(i,:),vertex(j,:))<=distance_threshold && check_path(vertex(i,:),vertex(j,:),img_bw)
            path_state(i,j)=1;
        end
    end
end

%% use A* to find a path
% list=[self_index,parent_index,g_cost,f_cost];
% fcost=gcost+hcost;
close_list=[];
open_list=[1,0,0,straight_distance(vertex(1,:),vertex(2,:))];
path_find=false;

while size(open_list,1)>0
    [~,min_cost_index]=min(open_list(:,4));
    current_node=open_list(min_cost_index,:);
    close_list=[close_list;current_node];
    open_list(min_cost_index,:)=[];
    
    if current_node(1,1)==2
        disp('find the best path , end search !');
        path_find=true;
        break;
    else
        child_node_index=[];
        for i=1:num_sample+2
            if path_state(current_node(1,1),i)==1
                child_node_index=[child_node_index,i];
            end
        end
        for i=child_node_index
            if ~ismember(i,open_list(:,1)) && ~ismember(i,close_list(:,1))
                child_node=A_star_list(vertex,i,current_node(1,1),current_node(1,1));
                open_list=[open_list;child_node];
            elseif ismember(i,open_list(:,1))
                exist_index=find(open_list(:,1)==i);
                new_fcost_node=A_star_list(vertex,i,current_node(1,1),current_node(1,1));
                if new_fcost_node(1,3)+current_node(1,3)<open_list(exist_index,3)
                    open_list(exist_index,:)=new_fcost_node;
                    open_list(exist_index,3)=new_fcost_node(1,3)+current_node(1,3);
                end
            end
        end
    end
end

%% paint the result path on the map
if ~path_find
    disp('no path finded , check your parm!');
else
    path=vertex(current_node(1,1),:);
    previous_index=current_node(1,2);
    % backtrack the path
    while previous_index~=0
        path=[vertex(previous_index,:);path];
        index=find(close_list(:,1)==previous_index);
        previous_index=close_list(index,2);
    end
    figure;
    paint_map(img_map,vertex,path_state,path);
end
