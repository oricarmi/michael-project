grid = createGrid(100,[30:100]',[1:30,70:100]');
figure;imagesc(grid);
writematrix(grid,'grid.xlsx');

