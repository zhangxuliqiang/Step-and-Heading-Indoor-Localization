
What is the problem?

- the app is not able to stream data to matlab anymore

- it seems that the EKF from the tutorial is not working correctly from log data
- it is not clear what is being logged exactly by the app

Steps that can be taken:

 1. search if app can be run in some kind of container on phone, since the app has worked before on the phone
	this is not possible

 2. find a different app for recording 
	a different app has been found that can both log and stream. Only how useful is it to stream information?


 3. find a different phone for the app to work on
	I can get a phone from manons office

 4. do a test to determine what the sampling frequency is
 - for a 10 second log with sampling set at 100 Hz around 10,000 data points are made
 - the same is applicable for 1 Hz, showing that that setting does nothing when logging
 - from the above results it seems that sampling occurs at 1000 Hz

Other Observations:

 - I think that there is a timing issue because the delta t is measured between all samples but maybe it should only be between the corresponding datasets

 ok this seems to have fixed the double angle displacement that was seen.

The next question is: is it worthwhile to continue with this setup or should you switch to the other app? The only difference at this point is that one is able to stream. I think it is fine to stick to the app you have at the moment

The xhat problem with the end of the array not being found has now. This was solved by defining the array explicitly.

IS IT WORTH IT TO WRITE THE WHOLE THING SELF?!! NOOOOOOO, WHY WOULD YOU DO THAT?! JUST GODDAMN CHECK IT POS! ADD A COMMENT THAT IT HAS BEEN CHECK OR SOMETHING! JESUS CRIST.
