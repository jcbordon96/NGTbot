var canvas = $('#msg')
var body = $('#body')
var menu = $('#menu')
var menuConf = $('#menuConf')
var menuControls = $('#menuControls')
var menuState = $('#menuState')
var openmenuConfBtn = $('#openmenuConfBtn')
var openmenuControlsBtn = $('#openmenuControlsBtn')
var backBtn1 = $('#backBtn1')
var backBtn2 = $('#backBtn1')
var saveConfigBtn = $('#saveConfigBtn')
var temp = $('#temp')
var humidity = $('#humidity')
var ammonia = $('#ammonia')
var lightStatus = $('#lightStatus')
var cameraStatus = $('#cameraStatus')
var autoStatus = $('#autoStatus')
var saveConfigBtn = $('#saveConfigBtn')
var rebootBtn = $('#rebootBtn')
var shutdownBtn = $('#shutdownBtn')
var forwardBtn = $('#forwardBtn')
var leftBtn = $('#leftBtn')
var rightBtn = $('#rightBtn')
var backBtn = $('#backBtn')


var robotState = {
    lights: false,
    camera: false,
    auto:true,
    temp:['0°', '0°', '0°'],
    humidity: '0',
    ammonia: '0'
}