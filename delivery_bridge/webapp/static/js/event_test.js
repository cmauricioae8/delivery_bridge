var toast_element = document.getElementById('toast_element');
var toast_title = document.getElementById('toast_title');
var toast_body = document.getElementById('toast_body');

var notification_toast_element = document.getElementById('notification_toast_element');
var notification_toast_title = document.getElementById('notification_toast_title');
var notification_toast_body = document.getElementById('notification_toast_body');

function send_message(message){
  console.log("Sending Message to server")
  socketio.emit('messages', data=message);
}
// funcion para atrapar mensajes del servidor
socketio.on("messages", (msg) => {
  console.log(msg.msg); // prints the message associated with the message
});
socketio.on("my_response", (msg) => {
  console.log(msg);
});
socketio.on("on_status_change", (msg) =>{
  console.log(msg.data);
  on_status_change_label.innerHTML = JSON.stringify(msg.data, undefined, 2);

  toast_title.innerHTML = "Status change for: " + Object.keys(msg.data)[0];
  var module_data = msg.data[Object.keys(msg.data)[0]];
  var text_body = "";
  Object.keys(module_data).forEach(key => {
    text_body += `<b>${key}</b>: ${JSON.stringify(module_data[key])}<br>`;
  })
  toast_body.innerHTML = text_body;
  
  var toast = new bootstrap.Toast(toast_element);
  toast.show();
})


/////////////////////////////////////////////////////////////////////////////////////////////
var vel_x = 0.1;
//var start_mode = document.getElementById("start_mode");

// ELEMENTS FOR CMD_VEL
var button_cmd_forward = document.getElementById("cmd_forward");
var button_cmd_backward = document.getElementById("cmd_backward");
var button_cmd_left = document.getElementById("cmd_left");
var button_cmd_right = document.getElementById("cmd_right");
var button_cmd_stop = document.getElementById("cmd_stop");

var position_element = document.getElementById('position');
// var on_mode_change_element = document.getElementById('on_mode_change');

// ELEMENTS FOR SET POSE
var button_pose_set = document.getElementById("pose_set");
var pose_position_x = document.getElementById("pose_position_x");
var pose_position_y = document.getElementById("pose_position_y");
var pose_orientation_z = document.getElementById("pose_orientation_z");

// ELEMENTS FOR BATTERY
var button_battery_get = document.getElementById("battery_get");
var button_battery_stop = document.getElementById("battery_stop");
var battery_as_event_checkbox = document.getElementById("battery_as_event_checkbox");
var battery_voltage = document.getElementById("battery_voltage");
var battery_percentage = document.getElementById("battery_percentage");

// ELEMENT FOR on_status_change EVENT
var on_status_change_label = document.getElementById("on_status_change_label");

/////////////////////////////////////////////////////////////////////////////////////////////



// ACTIONS FOR CMD_VEL
button_cmd_forward.addEventListener("click", function(){
  console.log("cmd_forward");
  // max: 0.3 m/s y 0.4 rad/s
  socketio.emit("cmd_vel", data={
    linear_x: 0.4,
    angular_z: 0.0
  });
});
button_cmd_backward.addEventListener("click", function(){
  console.log("cmd_backward");
  // max: 0.3 m/s y 0.4 rad/s
  socketio.emit("cmd_vel", data={
    linear_x: -0.4,
    angular_z: 0.0
  });
});
button_cmd_left.addEventListener("click", function(){
  console.log("cmd_left");
  // max: 0.3 m/s y 0.4 rad/s
  socketio.emit("cmd_vel", data={
    linear_x: 0.0,
    angular_z: 0.4
  });
});
button_cmd_right.addEventListener("click", function(){
  console.log("cmd_right");
  // max: 0.3 m/s y 0.4 rad/s
  socketio.emit("cmd_vel", data={
    linear_x: 0.0,
    angular_z: -0.4
  });
});
button_cmd_stop.addEventListener("click", function(){
  console.log("cmd_stop");
  socketio.emit("cmd_vel", data={
    linear_x: 0.0,
    angular_z: 0.0
  });
});

/////////////////////////////////////////////////////////////////////////////////////////////

// GET ROS EVENTS
// battery
socketio.on("battery", (msg) => {
  console.log("battery");
  battery_voltage.innerHTML = msg.data.voltage.toFixed(2) + " Volts";
  battery_percentage.innerHTML = msg.data.percentage.toFixed(2) + " %";
});
socketio.on('robot_pose', function(data) {
  //console.log(data);
  position_element.innerHTML = `(${data.position_x.toFixed(2)}, ${data.position_y.toFixed(2)}, ${data.orientation_z.toFixed(2)})`;
});
// socketio.on('on_mode_change', function(data) {
//   //console.log(data);
//   on_mode_change_element.innerHTML = data.mode;
// });