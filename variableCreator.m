 % Use a sub function which assigns variable in the caller function:
 function variableCreator ( newVar, variable )
   assignin ( 'caller', newVar, variable );
 end