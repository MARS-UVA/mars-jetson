import React, { useState, useEffect, useRef } from "react";
import { LineChart, Line, XAxis, YAxis, ResponsiveContainer, ReferenceLine } from "recharts";
import { Camera } from "lucide-react";

function LiveDataPanel() {
  const [chartData, setChartData] = useState(Array.from({ length: 1 }, (_, i) => ({
    time: i,
    value1: Math.random() * 100,
    value2: Math.random() * 100,
    value3: Math.random() * 100,
    value4: Math.random() * 100,
  })));
  const [lastDataPoint, setLastDataPoint] = useState(chartData[chartData.length - 1]);
  const [timestamp, setTimestamp] = useState(1);
  const round = num => (Math.round((num + Number.EPSILON) * 100) / 100).toFixed(2);
  const maxValue = Math.max(...chartData.map(data => Math.max(data.value1, data.value2, data.value3, data.value4)));

  const Chart = ({ dataKey }) => (
    <ResponsiveContainer width="100%" height={100}>
      <LineChart data={chartData}>
        <XAxis dataKey="time" />
        <YAxis />
        <Line type="monotone" dataKey={dataKey} stroke="#8884d8" dot={false} isAnimationActive={false} />
        {/* <ReferenceLine y={maxValue} stroke="red" strokeWidth={1} /> */}
      </LineChart>
    </ResponsiveContainer>
  );

  // Ref Hook - useRef for accessing a DOM element or mutable value
  

  useEffect(() => {

    const addNewData = () => {
      const newData = (l)=>{ return {
        time: l, // The next index as time
        value1: Math.random() * 100,
        value2: Math.random() * 100,
        value3: Math.random() * 100,
        value4: Math.random() * 100,
      }}
  
      // Set the new array (using the spread operator to create a new array)
      setChartData((prevData) => {
        let newDataPoint = newData(timestamp)
        setLastDataPoint(newDataPoint)
        setTimestamp((prevTime) => prevTime + 1)
        const newDataArray = [...prevData, newDataPoint];
        // Keep only the most recent 10 seconds of data
        return newDataArray.slice(-30);
      });
    };


    const interval = setInterval(addNewData, 1000);

    return () => clearInterval(interval);
  }, [timestamp]);

  
  
  // Render the component UI
  return (
    <>
      
      <div className="panel">
        {/* <h2 className="panel-title">Live Data Panel</h2> */}
        {/* <h3 className="panel-title">Charts</h3> */}
        <div className="chart-grid">
          <div className="chart-space"><Chart dataKey="value1" />Motor 1: {round(lastDataPoint["value1"])}</div>
          <div className="chart-space"><Chart dataKey="value2" />Motor 2: {round(lastDataPoint["value2"])}</div>
          <div className="chart-space"><Chart dataKey="value3" />Motor 3: {round(lastDataPoint["value3"])}</div>
          <div className="chart-space"><Chart dataKey="value4" />Motor 4: {round(lastDataPoint["value4"])}</div>
        </div>
      </div>
    </>
  );
}

export default LiveDataPanel;
