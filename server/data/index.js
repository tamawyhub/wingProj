var speedGauge=document.getElementById("speedVal");
var cs1Gauge=document.getElementById("c1Val");
var cs2Gauge=document.getElementById("c2Val");

var source = new EventSource("/events");
source.addEventListener("wingSensorsUpdate",function(e){
	var data=JSON.parse(e.data);
	speedGauge.setAttribute("data-value",data.speed);
	cs1Gauge.setAttribute("data-value",data.cs1);
	cs2Gauge.setAttribute("data-value",data.cs2);
},false);

function openTab(evt, tabName) {
  // Declare all variables
  var i, tabcontent, tablinks;

  // Get all elements with class="tabcontent" and hide them
  tabcontent = document.getElementsByClassName("tabcontent");
  for (i = 0; i < tabcontent.length; i++) {
    tabcontent[i].style.display = "none";
  }

  // Get all elements with class="tablinks" and remove the class "active"
  tablinks = document.getElementsByClassName("tablinks");
  for (i = 0; i < tablinks.length; i++) {
    tablinks[i].className = tablinks[i].className.replace(" active", "");
  }

  // Show the current tab, and add an "active" class to the button that opened the tab
  document.getElementById(tabName).style.display = "block";
  evt.currentTarget.className += " active";
}
