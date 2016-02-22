%input from the csv file
greatMatrix=csvread('C:\Users\ahuja\Desktop\3.csv');

num=38;%number of entries

%taking vectors and matrices from data
tempTime=greatMatrix(2:num,8:10);
sampleNumber=greatMatrix(2:num,3);
extTemperature=greatMatrix(2:num,6);
intTemperature=greatMatrix(2:num,4);
groundTemp=greatMatrix(2:num,23);
baroPressure=greatMatrix(2:num,5);
atmsPressure=greatMatrix(2:num,19);
groundBaroPressure=greatMatrix(2:num,22);
humidity=greatMatrix(2:num,7);
RAP=greatMatrix(2:num,18);
altitude=greatMatrix(2:num,14);
altFromPressureSensors=greatMatrix(2:num,20);
groundAltitude=greatMatrix(2:num,26);
longitude=greatMatrix(2:num,12);
groundLongitude=greatMatrix(2:num,25);
latitude=greatMatrix(2:num,13);
groundLatitude=greatMatrix(2:num,24);
heading=greatMatrix(2:num,15);
pitch=greatMatrix(2:num,16);
roll=greatMatrix(2:num,17);
DEW=greatMatrix(2:num,21);
Ax=greatMatrix(2:num,27);
Ay=greatMatrix(2:num,28);
Az=greatMatrix(2:num,29);
Gx=greatMatrix(2:num,30);
Gy=greatMatrix(2:num,31);
Gz=greatMatrix(2:num,32);
%need to be the same size, also somewhat sequential

%converting to one time vector
sz = size(sampleNumber);    %size of sampleNumber has to be the same as time.
time=zeros(sz);
for temp=1:(num-1)     %change accordingly
    time(temp,1)=3600*tempTime(temp,1)+60*tempTime(temp,2)+tempTime(temp,3);
end;

%ALL THESE GODDAMN PLOTS
plot(time, roll);
xlabel('time');
ylabel('roll');

plot(time,pitch);
xlabel('time');
ylabel('pitch');

plot(time,heading);
xlabel('time');
ylabel('heading');


plot(time, extTemperature);
xlabel('time');
ylabel('external temperature');

plot(time, intTemperature);
xlabel('time');
ylabel('internal temperature');

plot(time, groundTemp);
xlabel('time');
ylabel('ground temperature');

plot(time, baroPressure);
xlabel('time');
ylabel('Barometric Pressure');

plot(time, atmsPressure);
xlabel('time');
ylabel('Pressure (atms)');

plot(time, groundBaroPressure);
xlabel('time');
ylabel('Ground Pressure (atms)');

plot(time, RAP);
xlabel('time');
ylabel('Agricultural Viability');

plot(time, humidity);
xlabel('time');
ylabel('humidity');

plot(time, DEW);
xlabel('time');
ylabel('Dew point');

plot(time, altitude);
xlabel('time');
ylabel('altitude');

plot(time, altFromPressureSensors);
xlabel('time');
ylabel('Altitude from pressure sensors');

plot(time, groundAltitude);
xlabel('time');
ylabel('ground altitude');

plot(sampleNumber, roll);
xlabel('sample Number');
ylabel('roll');

plot(sampleNumber,pitch);
xlabel('sample Number');
ylabel('pitch');

plot(sampleNumber,heading);
xlabel('sample Number');
ylabel('heading');

plot(sampleNumber, extTemperature);
xlabel('sample Number');
ylabel('external temperature');

plot(sampleNumber, intTemperature);
xlabel('sample Number');
ylabel('internal temperature');

plot(sampleNumber, groundTemp);
xlabel('sample Number');
ylabel('ground temperature');

plot(sampleNumber, baroPressure);
xlabel('sample Number');
ylabel('Barometric Pressure');

plot(sampleNumber, atmsPressure);
xlabel('sample Number');
ylabel('Pressure (atms)');

plot(sampleNumber, groundBaroPressure);
xlabel('sample Number');
ylabel('Ground Pressure (atms)');

plot(sampleNumber, RAP);
xlabel('sample Number');
ylabel('Agricultural Viability');

plot(sampleNumber, humidity);
xlabel('sample Number');
ylabel('humidity');

plot(sampleNumber, DEW);
xlabel('sample Number');
ylabel('Dew point');

plot(sampleNumber, altitude);
xlabel('sample Number');
ylabel('altitude');

plot(sampleNumber, altFromPressureSensors);
xlabel('sample Number');
ylabel('Altitude from pressure sensors');

plot(sampleNumber, groundAltitude);
xlabel('sample Number');
ylabel('ground altitude');

plot(extTemperature, DEW);
xlabel('External temperature');
ylabel('Dew point');

plot(intTemperature, DEW);
xlabel('internal temperature');
ylabel('Dew point');

plot(groundTemp, DEW);
xlabel('ground temperature');
ylabel('Dew point');

geoshow(latitude, longitude);
xlabel('latitude');
ylabel('longitude');

geoshow(groundLatitude, groundLongitude);
xlabel('ground latitude');
ylabel('ground longitude');

%surf(heading, pitch, roll);
%xlabel('heading');
%ylabel('pitch');
%zlabel('roll');

% surf(Ax,Ay,Az);
% xlabel('Ax');
% ylabel('Ay');
% zlabel('Az');

surf(Gx,Gy,Gz);
xlabel('Gx');
ylabel('Gy');
zlabel('Gz');