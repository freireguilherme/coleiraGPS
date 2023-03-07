import 'package:flutter/material.dart';
import 'package:pet_tracker/models/pet.dart';

class PetDetails extends StatelessWidget {
  Pet pet;
  PetDetails({Key? key, required this.pet}) : super(key: key);

  @override
  Widget build(BuildContext context) {
    return Container(
      child: Wrap(
        children: [
          Image.network(
            pet.photo,
            height: 250,
            width: MediaQuery.of(context).size.width,
            fit: BoxFit.cover,
          ),
          Padding(
            padding: const EdgeInsets.all(8.0),
            child: Center(
              child: Text(
                pet.name,
                style:
                    const TextStyle(fontSize: 26, fontWeight: FontWeight.w600),
              ),
            ),
          ),
          Padding(
            padding: const EdgeInsets.only(bottom: 16),
            child: Center(
              child: Text('Código: xxxxx'),
            ),
          )
        ],
      ),
    );
  }
}
