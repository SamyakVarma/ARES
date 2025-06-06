
import React, { useState } from 'react';
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card';
import { Badge } from '@/components/ui/badge';

const MountedCameraFeed: React.FC = () => {
  const [feedStatus, setFeedStatus] = useState<'connected' | 'disconnected' | 'recording'>('connected');

  return (
    <Card className="military-panel h-full">
      <CardHeader className="pb-2">
        <div className="flex items-center justify-between">
          <CardTitle className="text-military-red text-sm">MOUNTED CAMERA</CardTitle>
          <div className="flex items-center space-x-2">
            <div className={`status-indicator ${
              feedStatus === 'connected' ? 'bg-military-green' : 
              feedStatus === 'recording' ? 'bg-military-red' : 'bg-gray-500'
            }`} />
            <Badge variant="outline" className="text-xs">
              {feedStatus === 'connected' ? 'LIVE' : 
               feedStatus === 'recording' ? 'REC' : 'OFFLINE'}
            </Badge>
          </div>
        </div>
      </CardHeader>
      <CardContent className="h-full pb-2">
        <div className="relative h-full bg-black rounded border border-military-border overflow-hidden">
          <div className="absolute inset-0 bg-gradient-to-br from-gray-900 to-black">
            <div className="absolute inset-0 bg-[url('data:image/svg+xml;base64,PHN2ZyB3aWR0aD0iNDAiIGhlaWdodD0iNDAiIHZpZXdCb3g9IjAgMCA0MCA0MCIgZmlsbD0ibm9uZSIgeG1sbnM9Imh0dHA6Ly93d3cudzMub3JnLzIwMDAvc3ZnIj4KPGRlZnM+CjxwYXR0ZXJuIGlkPSJncmlkIiB3aWR0aD0iNDAiIGhlaWdodD0iNDAiIHBhdHRlcm5Vbml0cz0idXNlclNwYWNlT25Vc2UiPgo8cGF0aCBkPSJNIDQwIDAgTCAwIDAgMCA0MCIgZmlsbD0ibm9uZSIgc3Ryb2tlPSIjMzMzIiBzdHJva2Utd2lkdGg9IjEiLz4KPC9wYXR0ZXJuPgo8L2RlZnM+CjxyZWN0IHdpZHRoPSIxMDAlIiBoZWlnaHQ9IjEwMCUiIGZpbGw9InVybCgjZ3JpZCkiIC8+Cjwvc3ZnPg==')] opacity-20" />
            
            <div className="absolute inset-0 flex items-center justify-center">
              <div className="relative">
                <div className="absolute w-8 h-px bg-military-red top-1/2 left-1/2 transform -translate-x-1/2 -translate-y-1/2" />
                <div className="absolute w-px h-8 bg-military-red top-1/2 left-1/2 transform -translate-x-1/2 -translate-y-1/2" />
                <div className="absolute w-3 h-3 border border-military-red rounded-full top-1/2 left-1/2 transform -translate-x-1/2 -translate-y-1/2" />
              </div>
            </div>
            
            <div className="absolute top-2 left-2 text-military-green text-xs font-mono">
              <div>CAM_02</div>
              <div>1920x1080</div>
              <div>30 FPS</div>
            </div>
            
            <div className="absolute top-2 right-2 text-military-red text-xs font-mono text-right">
              <div>{new Date().toLocaleTimeString()}</div>
              <div>ZOOM: 2.5x</div>
              <div>GIMBAL: ACTIVE</div>
            </div>
            
            <div className="absolute bottom-2 left-2 text-military-amber text-xs font-mono">
              <div>TARGET TRACKING: ACTIVE</div>
              <div>AUTO-FOCUS: ON</div>
            </div>
            
            <div className="absolute top-1/2 left-1/2 transform -translate-x-1/2 -translate-y-1/2 w-20 h-24 border-2 border-military-red animate-pulse">
              <div className="absolute -top-5 left-0 text-military-red text-xs">TARGET</div>
            </div>
          </div>
          
          {feedStatus === 'disconnected' && (
            <div className="absolute inset-0 bg-black bg-opacity-80 flex items-center justify-center">
              <div className="text-center">
                <div className="text-military-red text-lg font-bold mb-2">CONNECTION LOST</div>
                <div className="text-gray-400 text-sm">Attempting to reconnect...</div>
              </div>
            </div>
          )}
        </div>
      </CardContent>
    </Card>
  );
};

export default MountedCameraFeed;
