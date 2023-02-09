  const express = require('express')
  const { createServer } = require("http");
  const { Server } = require("socket.io")
  const mongoose = require("mongoose")
  const app = express()
  const MAX_LOCALIZATIONS = 7

  mongoose.connect('mongodb+srv://root:root@cluster0.1vgvw7r.mongodb.net/?retryWrites=true&w=majority');

  const httpServer = createServer(app);
  const socket = new Server(httpServer, {
    cors: {
      origin: "*"
    } 
  });

  const { Schema } = mongoose;

  const localizationSchema = new Schema({
    lat: Number,
    lng: Number
  });

  const Localization = mongoose.model('Localization', localizationSchema)

  app.use(express.json())

  socket.on("connection", (socket) => {
    console.log(socket, 'usuario conectado')
    sendPetLocalization()
  });
  socket.on("connect_error", (err) => {
    console.log(`connect_error due to ${err.message}`);
  });

  app.post('/', async (req, res) => { 
    const { lat, lng } = req.body;

    try {
      const localizationQuantity = await Localization.count({})
      if(localizationQuantity >= MAX_LOCALIZATIONS) {
        await Localization.findOneAndDelete(
          { },
          { "sort": { "date": -1 } }
        )
      }
      await Localization.create({ lat, lng})
      sendlastLocalization()
      res.send({message: "Sucesso", lat, lng})
    } catch(err) {
      console.log(err)
      res.send({ message: "Erro no sistema"})
    }


  })

  app.get('/', async (_, res) => {
    try {
      const results = await Localization.find({})
      res.send({message: "Sucesso", results })
    } catch(err) {
      console.log(err)
      res.send({ message: "Erro no sistema"})
    }
  })

  async function getLastLocalization() {
    try {
      const localization = await Localization.find({ created_at: -1 })
      return localization
    } catch(err) {
      console.log('err',err )
    }
  }


  


  async function sendPetLocalization() {
    console.log('oii')
    const lastLocalization = await getLastLocalization()
    socket.emit("pet_localization", lastLocalization)
  }

  httpServer.listen(3000) 
