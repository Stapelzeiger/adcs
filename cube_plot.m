function cube_plot(p,origin,X,Y,Z,orientation)
% CUBE_PLOT plots a cube with dimension of X, Y, Z.
%
% INPUTS:
% p      = patch object for drawing
% origin = set origin point for the cube in the form of [x,y,z].
% X      = cube length along x direction.
% Y      = cube length along y direction.
% Z      = cube length along z direction.
% orientation = rotation quaternion

vert = [1 1 0;
        0 1 0;
        0 1 1;
        1 1 1;
        0 0 1;
        1 0 1;
        1 0 0;
        0 0 0];

faces = [1 2 3 4;
         4 3 5 6;
         6 7 8 5;
         1 2 8 7;
         6 7 1 4;
         2 3 5 8];

color = [0, 1, 0;
         0, 0, 1;
         0, 0.5, 0;
         0, 0, 0.5;
         1, 0, 0;
         0.5, 0, 0];

cube = [vert(:,1)*X,vert(:,2)*Y,vert(:,3)*Z];
center = [X, Y, Z] * 0.5;
rcube = zeros(size(cube));
for vtx = 1:length(cube)
    cube(vtx, :) = rotate_by_quaternion(cube(vtx, :)' - center', orientation)' + origin;
end
set(p, 'faces', faces, 'vertices', cube, 'FaceVertexCData', color)
end
