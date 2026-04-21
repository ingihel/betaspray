function log(message) { print("[INFO] " + message); }

inputFile = getArgument();
isHeadless = (inputFile != ""); 

if (!isHeadless) {
    inputFile = File.openDialog("Select a climbing wall image to process");
    if (inputFile == "") {
        exit("No file selected. Script aborted.");
    }
}

log("loading " + inputFile);
open(inputFile);
// save image for overlaying later
originalTitle = getTitle(); 
run("Duplicate...", "title=working_copy");

log("do image cleanup (median filter)");
run("Median...", "radius=5");
log("subtract background");
run("Subtract Background...", "rolling=100 light separate sliding");
log("CLAHE");
run("Enhance Local Contrast (CLAHE)", "blocksize=300 histogram=256 maximum=3 mask=*None* fast_(less_accurate)");
log("median again");
run("Median...", "radius=5");
log("despeckle");
run("Despeckle");

log("apply HSV threshold...");
min=newArray(3); max=newArray(3); filter=newArray(3);

run("HSB Stack");
run("Convert Stack to Images");
selectWindow("Hue"); rename("0");
selectWindow("Saturation"); rename("1");
selectWindow("Brightness"); rename("2");

min[0]=0; max[0]=255; filter[0]="pass";
min[1]=0; max[1]=255; filter[1]="pass";
min[2]=0; max[2]=183; filter[2]="pass";

for (i=0;i<3;i++){
  selectWindow(""+i);
  setThreshold(min[i], max[i]);
  run("Convert to Mask");
  if (filter[i]=="stop")  run("Invert");
}

log("merge channels into mask...");
imageCalculator("AND create", "0","1");
imageCalculator("AND create", "Result of 0","2");

for (i=0;i<3;i++){
  selectWindow(""+i); close();
}
selectWindow("Result of 0"); close();
selectWindow("Result of Result of 0"); rename("Mask");

log("clean up mask morphology (Fill Holes, Open)...");
selectWindow("Mask");
run("Fill Holes");
setOption("BlackBackground", true);
run("Open");

// INGI: test
run("Watershed");

log("run Particle Analysis to extract big blob summary statistics...");
run("Set Measurements...", "area center redirect=None decimal=4");
run("Analyze Particles...", "size=150-Infinity display clear overlay add");


log("draw ROIs over the base image...");
selectWindow(originalTitle);
roiManager("Show All");
roiManager("Set Color", "red");
roiManager("Set Line Width", 2);
roiManager("Set Fill Color", "rgba(255, 0, 0, 80)"); 
run("Flatten");
outputImage = "/home/ingi/Highlighted_Holds.jpg";
log("saving thing to: " + outputImage);
saveAs("JPEG", outputImage);


outputFile = "/home/ingi/centroid_Positions.csv";
log("save to: " + outputFile);
saveAs("Results", outputFile);

if (isHeadless) {
    log("detected batch mode. exiting...");
    eval("script", "System.exit(0);");
} else {
    log("detected gui. leaving viewers open");
}
