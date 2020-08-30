function grid = createGrid(sizeGrid,sidewalkRows,sidewalkCols)
% create a grid with inputted size
if nargin<1
    disp('not enough inputs');
end
grid = zeros(sizeGrid);
grid(sidewalkRows,sidewalkCols) = 1;
end

