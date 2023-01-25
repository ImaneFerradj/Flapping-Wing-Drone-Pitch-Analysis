function generateVariableOnFly
   % lets tic/toc to compare the use of eval and assignin
   tic
   eval ( 'a = zeros(10,10);' )
   toc
   % an alternate method is to use a 
   % sub function which assigns vars in the caller function:
   tic
   variableCreator ( 'b', zeros(10,10) )
   toc
   % validate that a and b both exist and are the same:
   isequal ( a, b )  
 end
