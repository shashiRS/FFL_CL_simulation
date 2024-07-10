
function getMouse(event){
var i=0;

var x=event.clientX-8;
 x=((x/document.body.clientWidth)*tds.length-0.5).toFixed(2);

var y=event.clientY-8;
y=((y/document.body.clientHeight)*tds.length -0.5).toFixed(2);
x=parseFloat(tds[0].innerHTML)+parseFloat(x);
var str= "X pos: " + x ;//+ " %    Y pos: "+ y +"  %";
parent.document.getElementById("mousePos").innerHTML= str;}
 

 
