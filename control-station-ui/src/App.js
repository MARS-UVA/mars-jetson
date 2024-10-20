import React from "react";

import "./App.css";
import LiveDataPanel from "./Components/LiveDataPanel";
import GamepadPanel from "./Components/GamepadPanel";
import DriveStatePanel from "./Components/DriveStatePanel";
import WebcamPanel from "./Components/WebcamPanel";

const App = () => {

  return (
    <div className="app-container">
      {/* <h1 className="title">MARS Web UI</h1> */}


      <div className="content">
        <div className="left-panel">
          <GamepadPanel />

          <DriveStatePanel />
        </div>

        <div className="right-panel">
          <LiveDataPanel />
          <WebcamPanel/>
        </div>
      </div>
    </div>
  );
};

export default App;
