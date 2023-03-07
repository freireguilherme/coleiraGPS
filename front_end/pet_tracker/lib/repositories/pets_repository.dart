import 'package:flutter/material.dart';
import 'package:pet_tracker/models/pet.dart';

class PetsRepository extends ChangeNotifier {
  final List<Pet> pets = [
    Pet(
        name: 'Salsicha',
        photo:
            'https://guiaanimal.net/uploads/content/image/46760/AdobeStock_47432136.jpeg',
        lat: -9.39416,
        long: -40.5096)
  ];
}
