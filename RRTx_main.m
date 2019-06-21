%{
    Note: This is a code that I wrote in 2017. I've changed my style a lot.
    Please excuse for the badly written but still functional RRTx
    algorithm. This algorithm has been written from the following paper:
    https://pdfs.semanticscholar.org/0cac/84962d0f1176c43b3319d379a6bf478d50fd.pdf

    I plan to write this code from scratch in the future, but can't
    guarantee by when I will start and finish it. I suggest you to have a
    look at the paper and consider this code just as a reference as to how
    to code the algorithm.
%}


global node_pos parent edge_wt cost_goal neighbours line_handles rhs r_radius o_nodes discovered_obstacles video_name;
global goal_handles children  start_idx temp_edge_wt queue delta curr_node obstacle_cost filename file_index;

start = [1 1];       goal = [99 99];        epsilon = 5;         delta = 0;        ball_radius = (8)^2;
filename = 'RRTx_Map1_';            file_index = 1;              sample_frame = 60;
video_name = 'RRTx_Map1';
samples = 10000;
node_pos = NaN(samples,2);          parent = NaN(samples,1);                   edge_wt = cell(samples,1);          
neighbours = cell(samples,1);       cost_goal = Inf(samples,1);                rhs = Inf(samples,1);
children = cell(samples,1);         goal_handles = gobjects(samples,1);        queue = Inf(samples,3);          
rng(1);                             

figure,plot(start(1),start(2),'r.','MarkerSize',15);
hold on;
plot(goal(1),goal(2),'g.','MarkerSize',15);
h = gca;                            h.XLim = [0 100];                           h.YLim = [0 100];
i = 1;                              choose_goal_node_at_i = 600:50:800;         goal_chosen = 0;
node_pos(i,:) = goal(1:2);          cost_goal(i) = 0;                           limit_dia = 100;
rhs(i) = 0;                         discovered_obstacles = [];

time_limit = 70;                    pause_on = 0.001;                           line_handles = gobjects(samples,1);
goalAchieved=0;                     %tic;    
saveFrame(gcf);

while i<2000
%   'i' is the Node number
    i = i+1;
    if ~goal_chosen && any(i == choose_goal_node_at_i) 
        rand_point = start(1:2);
    else
        rand_point = limit_dia*rand(1,2);
    end
%   Calculate the distance of random point to all the nodes
    dist_points = pdist2(node_pos,rand_point,'squaredeuclidean');
    [sqDist,idx] = min(dist_points);
    
%   Bring the node at a distance of epsilon near to the closest node
    n_point = node_pos(idx,:);
    n_dist = realsqrt(sqDist);
    min_dist = min(n_dist,epsilon); % If the node is closer than epsilon, we have the same distance
    rand_point = n_point + (rand_point - n_point)*min_dist/n_dist;
    node_pos(i,:) = rand_point;
    
%   After bringing the node closer, now calculate the distances to all the nodes again
    all_edge_wt = pdist2(node_pos,rand_point,'squaredeuclidean');
    n_node_idx = all_edge_wt < ball_radius & all_edge_wt > 0 ;
    
%   Select the nodes that are within the ball radius. These become the neighbours. The 'n_' represents neighbours 
    n_nodes = find(n_node_idx);
    n_edge_wt = realsqrt(all_edge_wt(n_node_idx));
    n_rhs = rhs(n_node_idx);
    
%   Find the node with the smallest 'rhs'
    n_total_cost = n_edge_wt + n_rhs;
    [p_cost,p_idx] = min(n_total_cost);
    
%   Initialize the node properties
    parent(i) = n_nodes(p_idx);
    rhs(i) = p_cost;
    neighbours(i) = {n_nodes};
    edge_wt(i) = {n_edge_wt};
    
%   Now we have to tell the random points' neighbours that random point is
%   now a neighbour, also add the corresponding edge weight
    for j = 1:length(n_nodes)
            neighbours(n_nodes(j)) = {[neighbours{n_nodes(j)};i]};
            edge_wt(n_nodes(j)) = {[edge_wt{n_nodes(j)};n_edge_wt(j)]};
    end
    
%   Plot line from random point to its parent
    pause(pause_on);
    plot(rand_point(1),rand_point(2),'b.','MarkerSize',5);
    handle = line([node_pos(parent(i),1) node_pos(i,1)],[node_pos(parent(i),2) node_pos(i,2)],'Color',[0.2 0.2 0.2]);
    line_handles(i) = handle;
    
%   From now on it is from RRTx Paper 
    rewireNeighbours(i);
    reduceInconsistency();
    
%   If the goal has been reached, plot the goal line
    if rand_point == start(1:2) 
        start_idx = i;
        plotFrom(start_idx);
        goalAchieved = 1;
        goal_chosen = 1;
        previous_goal_cost = cost_goal(start_idx);
    end
    
%   If the cost to goal has decreased,then replot the goal line
    if  goalAchieved && (cost_goal(start_idx)<previous_goal_cost)
        delete(goal_handles);
        plotFrom(start_idx); 
        previous_goal_cost = cost_goal(start_idx);
    end
    
    if ~rem(i,sample_frame)
        saveFrame(gcf);
    end
    
end

saveFrame(gcf);

% Destroy the remaining unused space - This is done for faster execution
cost_goal(i+1:end) = [];               neighbours(i+1:end) = [];               edge_wt(i+1:end) = []; 
line_handles(i+1:end) = [];            parent(i+1:end) = [];                   node_pos(i+1:end,:) = [];
children(i+1:end)=[];                  temp_edge_wt = edge_wt;                 temp_children = children;

% Get all the children in a separate variable (cell)
for k = 1:i  
    children(k) = {find(parent==k)};
end

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 

% Now we perform D*Lite
obstacles = getMap();
obstacle_cost = Inf;
% obstacles = [0 30 80 20;20 70 80 20];       
r_radius = 10;                              sqr_radius = (r_radius)^2;                
curr_node = start_idx;                      prev_node = curr_node;
curr_ptr = gobjects(2,1);                   

o_nodes = getObstacleNodes(obstacles);      
% plotObstacles(obstacles);
pause_sim = 0.5;    
saveFrame(gcf);

while ~isnan(curr_node)
    
    r_dist = pdist2(node_pos,node_pos(curr_node,:),'squaredeuclidean');
    in_range = find(r_dist<sqr_radius);
    
%     o_nodes = getObstacleNodes(obstacles);
    updateObstacles(in_range);
        
    pause(pause_sim);
    next_node = parent(curr_node);
    line([node_pos(curr_node,1) node_pos(prev_node,1)],[node_pos(curr_node,2) node_pos(prev_node,2)],'Color','k','LineWidth',3);
        
    delete(curr_ptr);    
    curr_ptr = plotIndication(curr_ptr);
    
    plotIntersectObstacles(node_pos(curr_node,:), r_radius, obstacles);
    
    delete(goal_handles);
    plotFrom(curr_node);
    
    prev_node = curr_node;
    curr_node = next_node;
    
    saveFrame(gcf);
    
end

for j = 1:10
    saveFrame(gcf);
end

makeVideo();








% SubRoutines:

function plotIntersectObstacles(circle_center, radius, obstacles)

hold on;
t = 0.05:0.05:2*pi;
x = circle_center(1) + radius*cos(t);
y = circle_center(2) + radius*sin(t);

circle_poly = polyshape(x,y);

for i=1:length(obstacles)
   x_pos = [obstacles(i,1),obstacles(i,1)+obstacles(i,3),obstacles(i,1)+obstacles(i,3),obstacles(i,1)];
   y_pos = [obstacles(i,2),obstacles(i,2),obstacles(i,2)+obstacles(i,4),obstacles(i,2)+obstacles(i,4)];
   obs_poly = polyshape(x_pos,y_pos);
   
   polyout = intersect(circle_poly, obs_poly);
   plot(polyout, 'FaceColor', 'r', 'FaceAlpha', 1, 'EdgeColor', 'r');    
end

hold off;

end

function makeVideo()

global video_name file_index filename;

video_object = VideoWriter(video_name);
video_object.Quality = 100;
video_object.FrameRate = 4;
open(video_object);

for i = 1:file_index-1
    frameName = strcat(filename,num2str(i),'.jpg');
    read_frame= im2frame(imread(frameName));
    writeVideo(video_object, read_frame);   
end
close(video_object);

end

function saveFrame(handle)

global filename file_index;

full_name = strcat(filename,num2str(file_index),'.jpg');
saveas(handle,full_name);
file_index = file_index + 1;

end

function obstacles = getMap()

obstacles =[0,20,30,10;
            0,75,20,10;
            35,60,10,35;
            0,40,50,10;
            50,10,10,50;
            45,85,40,10;
            65,25,10,50;
            85,50,15,10;
            70,5,20,10;
            85,60,10,35] ;

end

function updateObstacles(nodes)

global o_nodes curr_node;

obstacle_inRange = intersect(nodes,o_nodes);

if any(obstacle_inRange)
    addNewObstacle(obstacle_inRange);
    propogateDescendants();
    verifyQueue(curr_node);
    reduceInconsistency_v2();
end

end

function addNewObstacle(nodes)

global neighbours obstacle_cost edge_wt parent temp_edge_wt;

    for i = 1:length(nodes)
        node = nodes(i);
        edge_wt(node) = {temp_edge_wt{node} + obstacle_cost};
        its_neighbours = neighbours{node};
        for j = 1:length(its_neighbours)
           neigh = its_neighbours(j);
           idx = find(neighbours{neigh}==node);
           t_edge = edge_wt{neigh};       
           t_edge(idx) = temp_edge_wt{neigh}(idx) + obstacle_cost;
           edge_wt(neigh) = {t_edge};
           if parent(neigh)==node
                verifyOrphan(neigh);
           end
        end
    end
    
    
end

function propogateDescendants()

global discovered_obstacles children neighbours parent cost_goal rhs line_handles;

nodes = discovered_obstacles;
all_children = [];
tic;
while ~isempty(nodes) && toc < 2
    nodes = unique(cell2mat(children(nodes)));  
    all_children = union(all_children,nodes);
end

discovered_obstacles = union(discovered_obstacles,all_children);

for i = 1:length(discovered_obstacles)
   spl_nodes = setdiff(neighbours{discovered_obstacles(i)},discovered_obstacles);
   if ~isempty(spl_nodes)
    cost_goal(spl_nodes) = Inf;
    verifyQueue(spl_nodes); 
   end
end

cost_goal(discovered_obstacles) = Inf;
rhs(discovered_obstacles) = Inf;
for i = 1:length(discovered_obstacles)
    its_parent = parent(discovered_obstacles(i));
     if ~isnan(its_parent)
         child = children{its_parent};
         child_less = child(child ~= discovered_obstacles(i));
         children(its_parent) = {child_less};
     end
end
delete(line_handles(discovered_obstacles));
parent(discovered_obstacles) = NaN;
discovered_obstacles = [];

end

function verifyOrphan(node)

global queue discovered_obstacles;

if ~isempty(find(queue(:,1)==node,1))
    queue(queue(:,1)==node,:) = Inf;
end
discovered_obstacles = union(discovered_obstacles,node);

end

function plotFrom(iter)
global node_pos parent goal_handles;
k = 1;
while iter~=1
    next_iter = parent(iter);
    goal_handles(k) = line([node_pos(iter,1) node_pos(next_iter,1)],[node_pos(iter,2) node_pos(next_iter,2)],'color','g','LineWidth',3);
    iter = next_iter;
    k = k+1;
end 
end

function verifyQueue(nodes)

global queue;

key = getKeys(nodes);
len = length(nodes);
for i=1:len
   idx = find(queue(:,1)==nodes(i)); 
    if ~isempty(idx)
        queue(idx,2:3) = key(i,:);
    else
        insertNodes(nodes(i))
    end
end

end

function insertNodes(nodes)

global queue;

key = getKeys(nodes);
queue(end-length(nodes)+1:end,:) = [nodes,key];
queue = sortrows(queue,[2 3 1]);

end

function key = getKeys(nodes)

global rhs cost_goal;

key = [min([cost_goal(nodes),rhs(nodes)],[],2), cost_goal(nodes)];

end

function top_value = popQueue()

global queue;

top_value = queue(1,:);
queue(1,:) = Inf;
queue = circshift(queue,-1);

end

function top_value = topQueue()

global queue;
top_value = queue(1);

end

function updateRHS(node)

global neighbours rhs edge_wt parent discovered_obstacles children;

neigh = neighbours{node};
edge = edge_wt{node};
banned_nodes = union(discovered_obstacles,neigh(parent(neigh)==node));
[~,idx] = setdiff(neigh,banned_nodes);
neigh = neigh(idx);
edge = edge(idx);

[updated_rhs,new_parent] = min(edge + rhs(neigh));
rhs(node) = updated_rhs;
parent(node) = neigh(new_parent);
children(neigh(new_parent)) = {unique([children{neigh(new_parent)};node])};
end

function rewireNeighbours(node)

global cost_goal rhs delta neighbours edge_wt parent line_handles node_pos children;

if cost_goal(node)-rhs(node) > delta || isnan(cost_goal(node)-rhs(node))
    neigh = neighbours{node};
    edge = edge_wt{node};
    idx = parent(neigh) ~= node;
    neigh = neigh(idx);
    edge = edge(idx);
    
    index = rhs(neigh) > edge + rhs(node);
    to_update = neigh(index);
    delete(line_handles(to_update));
    rhs(to_update) = edge(index) + rhs(node);
    parent(to_update) = node;
    children(node) = {unique([children{node};to_update])};
    for j = 1:length(to_update)
           handle = line([node_pos(to_update(j),1) node_pos(node,1)],[node_pos(to_update(j),2) node_pos(node,2)],'Color',[0.2 0.2 0.2]);
           line_handles(to_update(j)) = handle;
    end
        
    further_index = (cost_goal(to_update) - rhs(to_update) > delta) | (isnan(cost_goal(to_update) - rhs(to_update)));
    verifyQueue(to_update(further_index));
    
end

end

function reduceInconsistency()
    
global queue cost_goal rhs delta;

while any(any(~isinf(queue)))
   top = popQueue();
   
   if cost_goal(top(1)) - rhs(top(1)) > delta || isnan(cost_goal(top(1)) - rhs(top(1)))
        updateRHS(top(1));
        rewireNeighbours(top(1));      
   end
   cost_goal(top(1)) = rhs(top(1));
   
end

end

function reduceInconsistency_v2()
    
global queue cost_goal rhs delta curr_node;

while any(any(~isinf(queue))) && (keyLess(topQueue(),curr_node) || rhs(curr_node)~=cost_goal(curr_node) || cost_goal(curr_node)==Inf)
   top = popQueue();
   
   if cost_goal(top(1)) - rhs(top(1)) > delta || isnan(cost_goal(top(1)) - rhs(top(1)))
        updateRHS(top(1));
        rewireNeighbours(top(1));      
   end
   cost_goal(top(1)) = rhs(top(1));
   
end

end

function value = keyLess(top,start)

top_key = getKeys(top);        start_key = getKeys(start);
value = top_key(1)<start_key(1) & top_key(2)<start_key(2);

end

function plotObstacles(obstacles)

[rows,~] = size(obstacles);
hold on;
for i = 1:rows
    rectangle('Position',obstacles(i,:),'FaceColor','r');
end
hold off;

end

function curr_ptr = plotIndication(curr_ptr)
global node_pos curr_node r_radius;
    hold on;
    curr_ptr(1) = plot(node_pos(curr_node,1),node_pos(curr_node,2),'g.','MarkerSize',20);
    curr_ptr(2) = viscircles(node_pos(curr_node,:),r_radius,'LineStyle','--','Color','r');
    hold off;
end

function o_nodes = getObstacleNodes(obstacles)

global node_pos;
[x,~] = size(obstacles);
o_nodes = [];
for i=1:x
    within_range = node_pos>obstacles(i,1:2) & node_pos<obstacles(i,1:2)+obstacles(i,3:4);
    idx = within_range(:,1) & within_range(:,2);
    o_nodes = union(o_nodes,find(idx));
end


end
