classdef Client < handle
   properties
       m_Socket;
       m_Robot;
       m_nTimeStep;
   end
   methods
       function Disconnect(obj)
           obj.m_Robot.Clear();
       end
   end
end