import React, { useState, useEffect } from 'react';
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card';
import { Badge } from '@/components/ui/badge';

const CameraFeed: React.FC = () => {
  const [feedStatus, setFeedStatus] = useState<'connected' | 'disconnected' | 'recording'>('connected');

  useEffect(() => {
    if (feedStatus === 'disconnected') {
      const retry = setInterval(() => {
        const img = new Image();
        img.src = 'http://localhost:5000/video_feed?' + new Date().getTime();
        img.onload = () => {
          setFeedStatus('connected');
          clearInterval(retry);
        };
      }, 5000);

      return () => clearInterval(retry);
    }
  }, [feedStatus]);

  return (
   <Card className="military-panel w-full max-w-full aspect-video mx-auto flex flex-col h-80">
      <CardHeader className="pb-2">
        <div className="flex items-center justify-between">
          <CardTitle className="text-military-red text-sm">NAVIGATION CAMERA</CardTitle>
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

      <CardContent className="relative flex-1 overflow-hidden rounded border border-military-border bg-black">

        {/* Video Feed */}
        {feedStatus !== 'disconnected' && (
          <img
            src="http://localhost:5000/video_feed"
            alt="Camera Feed"
            onLoad={() => setFeedStatus('connected')}
            onError={() => setFeedStatus('disconnected')}
            className="absolute inset-0 w-full h-full object-contain z-0"
          />
        )}

        {/* Overlay */}
        <div className="absolute inset-0 z-10 pointer-events-none">
          <div className="absolute inset-0 bg-gradient-to-br from-gray-900 to-black opacity-30" />
          
          <div className="absolute inset-0 flex items-center justify-center">
            <div className="relative">
              <div className="absolute w-8 h-px bg-military-red top-1/2 left-1/2 transform -translate-x-1/2 -translate-y-1/2" />
              <div className="absolute w-px h-8 bg-military-red top-1/2 left-1/2 transform -translate-x-1/2 -translate-y-1/2" />
              <div className="absolute w-3 h-3 border border-military-red rounded-full top-1/2 left-1/2 transform -translate-x-1/2 -translate-y-1/2" />
            </div>
          </div>

          <div className="absolute top-2 left-2 text-military-green text-xs font-mono">
            <div>CAM_01</div>
            <div>1920x1080</div>
            <div>30 FPS</div>
          </div>

          <div className="absolute top-2 right-2 text-military-red text-xs font-mono text-right">
            <div>{new Date().toLocaleTimeString()}</div>
            <div>GPS: 40.7128, -74.0060</div>
            <div>ZOOM: 1.0x</div>
          </div>

          <div className="absolute bottom-2 left-2 text-military-amber text-xs font-mono">
            <div>TARGET LOCK: DISABLED</div>
            <div>TRACKING: STANDBY</div>
          </div>

          <div className="absolute top-1/3 left-1/4 w-16 h-20 border-2 border-military-red animate-pulse">
            <div className="absolute -top-5 left-0 text-military-red text-xs">PERSON</div>
          </div>

          <div className="absolute bottom-1/3 right-1/3 w-12 h-8 border-2 border-military-amber">
            <div className="absolute -top-5 left-0 text-military-amber text-xs">OBJECT</div>
          </div>
        </div>

        {/* Connection Lost Overlay */}
        {feedStatus === 'disconnected' && (
          <div className="absolute inset-0 bg-black bg-opacity-80 flex items-center justify-center z-20">
            <div className="text-center">
              <div className="text-military-red text-lg font-bold mb-2">CONNECTION LOST</div>
              <div className="text-gray-400 text-sm">Attempting to reconnect...</div>
            </div>
          </div>
        )}
      </CardContent>
    </Card>
  );
};

export default CameraFeed;
