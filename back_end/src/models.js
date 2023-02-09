const mongoose = require("mongoose")
const { Schema } = mongoose;

const localizationSchema = new Schema({
  lat: Number,
  lng: Number
});

const Localization = mongoose.model('Localization', localizationSchema)

module.exports = Localization