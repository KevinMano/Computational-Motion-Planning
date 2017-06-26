function [distance] = DistanceCalc(current_node, neighbour_nodex, neighbour_nodey)
    distance = abs(current_node(1) - neighbour_nodex) + abs(current_node(2) - neighbour_nodey);
end
