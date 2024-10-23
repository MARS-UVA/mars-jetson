import React, { useState, useEffect, useRef } from "react";
import { Camera } from "lucide-react";

// This component renders a panel with a webcam feed (currently showing laptop webcam)
function WebcamPanel() {

    const videoRef = useRef(null);

    useEffect(() => {
        if (navigator.mediaDevices && navigator.mediaDevices.getUserMedia) {
          navigator.mediaDevices
            .getUserMedia({ video: true })
            .then((stream) => {
              if (videoRef.current) {
                videoRef.current.srcObject = stream;
              }
            })
            .catch((error) => {
              console.error("Error accessing webcam:", error);
            });
        }
      }, []);



    return (
    <>
      <div className="panel">
        <div className="webcam-container">
          <video ref={videoRef} autoPlay playsInline muted className="webcam" />
          <div className="camera-icon">
            <Camera size={20} />
          </div>
        </div>
      </div>
    </>
    )
}

export default WebcamPanel;
