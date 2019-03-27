
% calculate_vector calculates the next position and the vectors computed by
% using the current position and the next position.
function [V, next_x, next_y, vec_x, vec_y, bearing] = calculate_vector(curr_x, curr_y, coord_matrix)
n = size(coord_matrix, 1);
x = 0;
y = 0;
vec12 = 0; % vector between pos1 and pos2
[pos1, pos2] = closest_pos(curr_x, curr_y, coord_matrix);
last_pos = [0, 0];
next_pos = [0, 0];
heading = [0, 0]; % this is the current heading the car was going towards
for i = 1:n
    if isequal(coord_matrix(i,:),pos1) || isequal(coord_matrix(i,:),pos2)
        if isequal(coord_matrix(i,:),pos1) 
            % this means pos1 occured faster than pos2 in the coord_matrix
            x = pos2(1);
            y = pos2(2);
            vec12 = [(pos2(1) - pos1(1)) (pos2(2) - pos1(2))];
            last_pos = pos2;
            break
        elseif isequal(coord_matrix(i,:),pos2) 
            % this means pos2 occured faster than pos1 in the coord_matrix
            x = pos1(1);
            y = pos1(2);
            vec12 = [(pos1(1) - pos2(1)) (pos1(2) - pos2(2))];
            last_pos = pos1;
            break
        end
    end
end
heading = last_pos - [curr_x curr_y];
disp(pos1)
disp(pos2)
next_x = x;
next_y = y;
vec_x = next_x - curr_x;
vec_y = next_y - curr_y;
% this case is coded to prevent the last point to first point heading
% error. I just made sure that whenever the two closest points are the
% first and last point of the coord_matrix, just head towards the first
% point.
if (isequal(pos1, coord_matrix(1,:)) && isequal(pos2, coord_matrix(n,:)))||(isequal(pos2, coord_matrix(1,:)) && isequal(pos1, coord_matrix(n,:)))
    next_pos = coord_matrix(1,:);
    next_x = next_pos(1);
    next_y = next_pos(2);
    vec_x = next_x - curr_x;
    vec_y = next_y - curr_y;
end
% the current position are usually between two closest positions but 
% sometimes the positions are before the current position. 
% Thus this case is needed to prevent the car having the wrong heading.
if ~vec_sign_compare([vec_x vec_y], vec12) 
    if get_index(coord_matrix, last_pos) ~= n
        % if case is needed to check edge case when the index of last_pos
        % is n.
        next_pos = coord_matrix(get_index(coord_matrix, last_pos)+1,:);
    else 
        next_pos = coord_matrix(1,:);
    end
    next_x = next_pos(1);
    next_y = next_pos(2);
    vec_x = next_x - curr_x;
    vec_y = next_y - curr_y;
    heading = [curr_x curr_y] - last_pos;
end
bearing = acos(dot(heading, [vec_x vec_y])/(norm(heading)*norm([vec_x vec_y])));
% this bearing used the function cos(theta) = (u dot v) / (|u|*|v|) to get
% the angle between two vectors
cross_product = cross([heading 0], [vec_x vec_y 0]);
% I used cross product to check if the car is going right or left
if cross_product(3) < 0 
    bearing = -1*bearing;
end
V = assign_velocity([next_x next_y], coord_matrix);
end


% assign_velocity assignes the velocity the car have to go.
% the velocity is constant when the car goes straight and it reduces while
% curving. I wasn't sure how to check if the car is in a curving state or
% in a straight state, so for now, I just look at the coord_matrix and see
% where the car needs to be curving or going straight.
function velocity = assign_velocity(pos, coord_matrix) 
pos_i = get_index(coord_matrix, pos);
ones_dig = mod(pos_i, 10); % ones degree, example) 4526's ones degree is 6.
if (ones_dig <= 5 && ones_dig ~= 0) 
    velocity = 4;
else 
    if ones_dig <= 8
        velocity = 4 - (ones_dig - 5)^0.3;
    else 
        velocity = 4 - (10 - ones_dig)^0.3;
    end
end
end


% closest_pos gives the two closest positions in the coord_matrix from 
% coordinate (curr_x, curr_y)
function [pos1, pos2] = closest_pos(curr_x, curr_y, coord_matrix)
% curr_x: current x
% curr_y: current y
% coord_matrix is the matrix that contains all the interpolation positions
% pos1 is the position of closest and pos2 is the position of second
% closest
n = size(coord_matrix, 1); % n represents the number of points we have
closest_d = distance(coord_matrix(1,:), [curr_x, curr_y]);
pos1 = coord_matrix(1,:);
pos2 = coord_matrix(2,:);
for i = 1:n
    if distance(coord_matrix(i,:), [curr_x curr_y]) < closest_d
        closest_d = distance(coord_matrix(i,:), [curr_x curr_y]);
        pos2 = pos1;
        pos1 = coord_matrix(i,:);
    end
end
end


% distance calculates the distance between two coordinates coord1 and
% coord2
% coord1 and coord2 are both 1 x 2 matrices
function d = distance(coord1, coord2) 
d = sqrt((coord2(1)-coord1(1))^2+(coord2(2)-coord1(2))^2);
% d is just distance = sqrt((x2-x1)^2+(y2-y1)^2)
end

% vec-sign_compare comapares the sign of two vectors and check
% whether the heading is correct or not. When a vector has a coordinate 
% which is zero, we just ignore it.
function bool = vec_sign_compare(vec1, vec2) 
vec1_sign = give_sign(vec1);
vec2_sign = give_sign(vec2);
if ~((vec1(1) == 0) || (vec1(2) == 0) || (vec2(1) ==0) || (vec2(2) == 0)) 
    if isequal(vec1_sign, vec2_sign) 
        bool = 1;
    else
        bool = 0;
    end
else
    if isequal(vec1_sign(1), vec2_sign(1)) || isequal(vec1_sign(2), vec2_sign(2))
        bool = 1;
    else
        bool = 0;
    end
end
   

end

% give_sign converts the vector to its normalized heading
% positive is noted as 1  and negative is noted as -1 and 0 is noted as 0.
function sign = give_sign(vec) 
sign = [0 0];
if vec(1) < 0
    sign(1) = -1;
elseif vec(1) > 0
    sign(1) = 1;
end
if vec(2) < 0
    sign(1) = -1;
elseif vec(2) > 0
    sign(2) = 1;
end
end

% get_index gives the row index that pos is in by looking through coor_matrix.
function index = get_index(coord_matrix, pos) 
n = size(coord_matrix, 1);
for i = 1:n
    if isequal(pos, coord_matrix(i,:))
        index = i;
    end
end
end
