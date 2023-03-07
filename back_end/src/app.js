const express = require("express");
const { createServer } = require("http");
const axios = require("axios");

const app = express();
const MAX_LOCALIZATIONS = 7;
const thingspeakApiUrl = process.env.THINGSPEAK_URL;
const httpServer = createServer(app);

app.use(express.json());

app.post("/", async (req, res) => {
  const { lat, lng } = req.body;

  try {
    const localizationQuantity = await Localization.count({});
    if (localizationQuantity >= MAX_LOCALIZATIONS) {
      await Localization.findOneAndDelete({}, { sort: { date: -1 } });
    }
    await Localization.create({ lat, lng });
    sendPetLocalization();
    res.send({ message: "Sucesso", lat, lng });
  } catch (err) {
    console.log(err);
    res.send({ message: "Erro no sistema" });
  }
});

app.get("/", async (_, res) => {
  try {
    const result = await axios.get(thingspeakApiUrl);

    const localizations = result.data.feeds
      .slice(-7)
      .reverse()
      .map((localization) => {
        const { field1, field2, entry_id } = localization;
        return { lat: field1, lng: field2, id: entry_id };
      });

    res.send({ message: "Sucesso", localizations });
  } catch (err) {
    console.log(err);
    res.send({ message: "Erro no sistema" });
  }
});

async function getLastLocalization() {
  try {
    const localization = await Localization.find({ created_at: -1 });
    return localization;
  } catch (err) {
    console.log("err", err);
  }
}

async function sendPetLocalization() {
  const lastLocalization = await getLastLocalization();
  socket.emit("pet_localization", lastLocalization);
}

httpServer.listen(3000);
