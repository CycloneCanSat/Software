# Sensor Library
---

This will implement the interface with:
  * The MS5637 temperature and pressure sensor
  * The HYT-271 temperature and humidity sensor


This library assumes that the user has already included the Wire library.

---


## Class
object.pollMS5637() - void function. Reads MS5637 and updates object variables.


object.pollHYT271() - void function. Read IST HYT-271 and updates object variables.


object.initialise() - does what it says on the tin. Only required for MS5637.
