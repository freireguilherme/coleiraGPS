import 'package:flutter/material.dart';
import 'package:google_maps_flutter/google_maps_flutter.dart';
import 'package:provider/provider.dart';
import 'package:pet_tracker/controllers/pets_controller.dart';

final appKey = GlobalKey();

class PetTrackerPage extends StatelessWidget {
  const PetTrackerPage({super.key});

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      key: appKey,
      appBar: AppBar(
        title: const Text('Pet Tracker'),
      ),
      body: MediaQuery(
        data: MediaQuery.of(context).copyWith(),
        child: ChangeNotifierProvider<PetsController>(
          create: (context) => PetsController(),
          child: Builder(builder: (context) {
            final local = context.watch<PetsController>();

            return GoogleMap(
              initialCameraPosition: CameraPosition(
                target: LatLng(local.lat, local.long),
                zoom: 14,
              ),
              zoomControlsEnabled: true,
              mapType: MapType.normal,
              myLocationEnabled: true,
              onMapCreated: local.onMapCreated,
              markers: local.markers,
            );
          }),
        ),
      ),
    );
  }
}
