function paint_map( map,vertex,path_state,path)
imshow(map);
for i=1:size(vertex,1)
    rectangle('position',[vertex(i,2)-3,vertex(i,1)-3,6,6],'cURVATURE',[1,1],'FaceColor','k');
end
for i=1:size(vertex,1)
    for j=1:size(vertex,1)
        if path_state(i,j)==1
            line([vertex(i,2),vertex(j,2)],[vertex(i,1),vertex(j,1)]);
        end
    end
end
line(path(:,2),path(:,1),'color','r','linewidth',2);
end

