
import React from 'react';
import { Badge } from '@/components/ui/badge';

interface SystemStatusProps {
  connectionStatus: string;
}

const SystemStatus: React.FC<SystemStatusProps> = ({ connectionStatus }) => {
  const getStatusColor = (status: string) => {
    switch (status) {
      case 'connected': return 'bg-military-green';
      case 'connecting': return 'bg-military-amber';
      default: return 'bg-military-red';
    }
  };

  return (
    <div className="flex items-center space-x-3 text-xs">
      {/* Connection Status */}
      <div className="flex items-center space-x-2">
        <div className={`status-indicator ${getStatusColor(connectionStatus)}`} />
        <div className="text-right">
          <div className="text-gray-500">CONNECTION</div>
          <div className="text-white font-semibold">{connectionStatus.toUpperCase()}</div>
        </div>
      </div>
      
      {/* System Time */}
      <div className="text-right">
        <div className="text-gray-500">TIME</div>
        <div className="text-military-red font-mono text-glow">
          {new Date().toLocaleTimeString()}
        </div>
      </div>
      
      {/* Operation Status */}
      <Badge variant="outline" className="bg-military-panel border-military-green text-military-green text-glow">
        OPERATIONAL
      </Badge>
    </div>
  );
};

export default SystemStatus;
