var express = require('express');
var app = express();
const bodyParser = require('body-parser');

var tdsVal, phVal, doVal, tempVal;

app.use(bodyParser.json());

app.get('/', function (req, res) {
   res.sendFile( __dirname + "/" + "index.html" );
})

app.get('/sensor', function(req, res) {
   res.send(JSON.stringify({
      tds : tdsVal,
      ph :phVal,
      do : doVal,
      temp : tempVal
   }))
})

app.post('/recv-sensor', (req, res) => {
   var reqBody = req.body;
   
   tdsVal = reqBody.tds;
   phVal = reqBody.ph;
   doVal = reqBody.do;
   tempVal = reqBody.temp;

   res.status(200).send(JSON.stringify({
       status : 'success'
   }));
})

app.listen(5000);