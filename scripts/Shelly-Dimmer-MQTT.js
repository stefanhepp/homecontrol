// Script: MQTT Topics
//
// This script is used for Shelly Dimmer Gen3, to publish output state and brightness as MQTT topics.

let CONFIG = {
  lightId: 0,
  buttonInput: "input:0",
  topic: "shelly/tv",
  configTopic: "shelly/tv",
};

let publishedOutputState = false;
let publishedBrightness = 0;

function publishStatus(publishState) {
  if (MQTT.isConnected()) {
    let status = Shelly.getComponentStatus("light", CONFIG.lightId);
    if (publishState) {
      publishedOutputState = status.output;
      publishedBrightness = status.brightness;
      MQTT.publish(CONFIG.topic + "/output", status.output ? "on" : "off", 0, true);
      MQTT.publish(CONFIG.topic + "/brightness", String(status.brightness), 0, true);
    }
    MQTT.publish(CONFIG.topic + "/temperature", String(status.temperature.tC), 0, true);
    MQTT.publish(CONFIG.topic + "/apower", String(status.apower), 0, true);
    MQTT.publish(CONFIG.topic + "/current", String(status.current), 0, true);
    MQTT.publish(CONFIG.topic + "/voltage", String(status.voltage), 0, true);
    MQTT.publish(CONFIG.topic + "/status", JSON.stringify(status), 0, true);
  }
}

function publishConfig() {
  if (MQTT.isConnected()) {
    let config = Shelly.getComponentConfig("light", CONFIG.lightId);
    MQTT.publish(CONFIG.configTopic + "/config", JSON.stringify(config));
  }
}

function connectMQTT() {
  if (MQTT.isConnected()) {
    console.log("MQTT connected!");
    publishStatus(true);
    publishConfig();

    MQTT.subscribe(CONFIG.topic + "/output", function(topic, message) {
      let output = (message === "on");
      let status = Shelly.getComponentStatus("light", CONFIG.lightId);
      if (output != status.output && output != publishedOutputState) {
        Shelly.call("Light.Set", {id: CONFIG.lightId, on: output});
      }
      });
    MQTT.subscribe(CONFIG.topic + "/brightness", function(topic, message) {
      let brightness = parseInt(message);
      let status = Shelly.getComponentStatus("light", CONFIG.lightId);
      if (brightness != status.brightness && brightness != publishedBrightness) {
        try {
          Shelly.call("Light.Set", {id: CONFIG.lightId, brightness: brightness});
        } catch (e) {
          console.log("Error:", e.message);
        }
      }
      });
  }
}

function handleEvent(event) {
  if (event.component === CONFIG.buttonInput) {
    console.log("Button: ", event.info.event);
    if (event.info.event === "single_push") {
      console.log("Button - toggling light");
    }
    else if (event.info.event === "double_push") {
      let status = Shelly.getComponentStatus("light", CONFIG.lightId);
    }
  }
}

function handleStatus(status) {
  // Check for motion sensor updates
  if (status.component === "light:" + CONFIG.lightId) {
    if ("output" in status.delta) {
      publishedOutputState = status.delta.output;
      MQTT.publish(CONFIG.topic + "/output", status.delta.output ? "on" : "off", 0, true);
    }
    if ("brightness" in status.delta) {
      publishedBrightness = status.delta.brightness;
      MQTT.publish(CONFIG.topic + "/brightness", String(status.delta.brightness), 0, true);
    }

    publishStatus(false);

    // console.log("Got status", JSON.stringify(status));
  }
}

//Shelly.addEventHandler(handleEvent);
Shelly.addStatusHandler(handleStatus);
MQTT.setConnectHandler(connectMQTT);

// subscribe and publish, in case MQTT is already connected when this script runs.
connectMQTT();

