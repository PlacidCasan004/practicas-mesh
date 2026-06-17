/*M!999999\- enable the sandbox mode */ 
-- MariaDB dump 10.19-11.8.6-MariaDB, for debian-linux-gnu (x86_64)
--
-- Host: localhost    Database: sensores
-- ------------------------------------------------------
-- Server version	11.8.6-MariaDB-ubu2404

/*!40101 SET @OLD_CHARACTER_SET_CLIENT=@@CHARACTER_SET_CLIENT */;
/*!40101 SET @OLD_CHARACTER_SET_RESULTS=@@CHARACTER_SET_RESULTS */;
/*!40101 SET @OLD_COLLATION_CONNECTION=@@COLLATION_CONNECTION */;
/*!40101 SET NAMES utf8mb4 */;
/*!40103 SET @OLD_TIME_ZONE=@@TIME_ZONE */;
/*!40103 SET TIME_ZONE='+00:00' */;
/*!40014 SET @OLD_UNIQUE_CHECKS=@@UNIQUE_CHECKS, UNIQUE_CHECKS=0 */;
/*!40014 SET @OLD_FOREIGN_KEY_CHECKS=@@FOREIGN_KEY_CHECKS, FOREIGN_KEY_CHECKS=0 */;
/*!40101 SET @OLD_SQL_MODE=@@SQL_MODE, SQL_MODE='NO_AUTO_VALUE_ON_ZERO' */;
/*M!100616 SET @OLD_NOTE_VERBOSITY=@@NOTE_VERBOSITY, NOTE_VERBOSITY=0 */;

--
-- Table structure for table `humidity_measurements`
--

DROP TABLE IF EXISTS `humidity_measurements`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8mb4 */;
CREATE TABLE `humidity_measurements` (
  `id` int(11) NOT NULL AUTO_INCREMENT,
  `nodo` varchar(50) NOT NULL,
  `valor` float NOT NULL,
  `unidad` varchar(10) DEFAULT '%',
  `relay` int(11) DEFAULT NULL,
  `mode` varchar(50) DEFAULT NULL,
  `layer` int(11) DEFAULT NULL,
  `fecha` timestamp NULL DEFAULT current_timestamp(),
  PRIMARY KEY (`id`)
) ENGINE=InnoDB AUTO_INCREMENT=32 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_uca1400_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `humidity_measurements`
--

SET @OLD_AUTOCOMMIT=@@AUTOCOMMIT, @@AUTOCOMMIT=0;
LOCK TABLES `humidity_measurements` WRITE;
/*!40000 ALTER TABLE `humidity_measurements` DISABLE KEYS */;
INSERT INTO `humidity_measurements` VALUES
(1,'nodo_bme_rele_1',32.36,'%',0,'MANUAL_OFF',2,'2026-04-28 10:58:16'),
(2,'nodo_bme_rele_1',32.41,'%',0,'MANUAL_OFF',2,'2026-04-28 10:58:46'),
(3,'nodo_bme_rele_1',32.45,'%',0,'MANUAL_OFF',2,'2026-04-28 10:59:17'),
(4,'nodo_bme_rele_1',32.37,'%',0,'MANUAL_OFF',2,'2026-04-28 10:59:47'),
(5,'nodo_bme_rele_1',32.3,'%',0,'MANUAL_OFF',2,'2026-04-28 11:00:20'),
(6,'nodo_bme_rele_1',32.3,'%',0,'MANUAL_OFF',2,'2026-04-28 11:00:20'),
(7,'nodo_bme_rele_1',32.3,'%',0,'MANUAL_OFF',2,'2026-04-28 11:00:20'),
(8,'nodo_bme_rele_1',32.28,'%',0,'MANUAL_OFF',2,'2026-04-28 11:00:48'),
(9,'nodo_bme_rele_1',32.26,'%',0,'MANUAL_OFF',2,'2026-04-28 11:01:18'),
(10,'nodo_bme_rele_1',32.27,'%',0,'MANUAL_OFF',2,'2026-04-28 11:01:48'),
(11,'nodo_bme_rele_1',32.27,'%',0,'MANUAL_OFF',2,'2026-04-28 11:02:19'),
(12,'nodo_bme_rele_1',32.29,'%',0,'MANUAL_OFF',2,'2026-04-28 11:02:49'),
(13,'nodo_bme_rele_1',32.26,'%',0,'MANUAL_OFF',2,'2026-04-28 11:03:19'),
(14,'nodo_bme_rele_1',32.27,'%',0,'MANUAL_OFF',2,'2026-04-28 11:03:50'),
(15,'nodo_bme_rele_1',32.27,'%',0,'MANUAL_OFF',2,'2026-04-28 11:04:21'),
(16,'nodo_bme_rele_1',32.25,'%',0,'MANUAL_OFF',2,'2026-04-28 11:04:51'),
(17,'nodo_bme_rele_1',32.34,'%',0,'MANUAL_OFF',2,'2026-04-28 11:05:21'),
(18,'nodo_bme_rele_1',32.26,'%',0,'MANUAL_OFF',2,'2026-04-28 11:05:52'),
(19,'nodo_bme_rele_1',32.31,'%',0,'MANUAL_OFF',2,'2026-04-28 11:06:22'),
(20,'nodo_bme_rele_1',32.33,'%',0,'MANUAL_OFF',2,'2026-04-28 11:06:52'),
(21,'nodo_bme_rele_1',32.37,'%',0,'MANUAL_OFF',2,'2026-04-28 11:07:22'),
(22,'nodo_bme_rele_1',32.4,'%',0,'MANUAL_OFF',2,'2026-04-28 11:07:53'),
(23,'nodo_bme_rele_1',32.44,'%',0,'MANUAL_OFF',2,'2026-04-28 11:08:23'),
(24,'nodo_bme_rele_1',32.41,'%',0,'MANUAL_OFF',2,'2026-04-28 11:08:54'),
(25,'nodo_bme_rele_1',32.43,'%',0,'MANUAL_OFF',2,'2026-04-28 11:09:24'),
(26,'nodo_bme_rele_1',32.43,'%',0,'MANUAL_OFF',2,'2026-04-28 11:09:55'),
(27,'nodo_bme_rele_1',32.42,'%',0,'MANUAL_OFF',2,'2026-04-28 11:10:24'),
(28,'nodo_bme_rele_1',32.51,'%',0,'MANUAL_OFF',2,'2026-04-28 11:10:55'),
(29,'nodo_bme_rele_1',32.44,'%',0,'MANUAL_OFF',2,'2026-04-28 11:11:25'),
(30,'nodo_bme_rele_1',32.43,'%',0,'MANUAL_OFF',2,'2026-04-28 11:11:55'),
(31,'nodo_bme_rele_1',32.43,'%',0,'MANUAL_OFF',2,'2026-04-28 11:12:26');
/*!40000 ALTER TABLE `humidity_measurements` ENABLE KEYS */;
UNLOCK TABLES;
COMMIT;
SET AUTOCOMMIT=@OLD_AUTOCOMMIT;

--
-- Table structure for table `nodos`
--

DROP TABLE IF EXISTS `nodos`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8mb4 */;
CREATE TABLE `nodos` (
  `mac` varchar(17) NOT NULL,
  `tipo_nodo` varchar(50) NOT NULL,
  `sensores_json` longtext CHARACTER SET utf8mb4 COLLATE utf8mb4_bin NOT NULL CHECK (json_valid(`sensores_json`)),
  `topic_pub` varchar(255) NOT NULL,
  `topic_sub` varchar(255) NOT NULL,
  PRIMARY KEY (`mac`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_uca1400_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `nodos`
--

SET @OLD_AUTOCOMMIT=@@AUTOCOMMIT, @@AUTOCOMMIT=0;
LOCK TABLES `nodos` WRITE;
/*!40000 ALTER TABLE `nodos` DISABLE KEYS */;
INSERT INTO `nodos` VALUES
('14:c1:9f:29:e4:58','camara','[]','mesh/data/14c19f29e458','mesh/cmd/14c19f29e458'),
('24:dc:c3:8d:7b:d8','router','[]','mesh/data/24dcc38d7bd8','mesh/cmd/24dcc38d7bd8'),
('24:dc:c3:8e:2f:48','router','[]','mesh/data/24dcc38e2f48','mesh/cmd/24dcc38e2f48'),
('24:dc:c3:92:cc:94','router','[]','mesh/data/24dcc392cc94','mesh/cmd/24dcc392cc94'),
('84:0d:8e:37:79:28','sensor_actuador','[{\"tipo\":\"temperatura\"},{\"tipo\":\"humedad\"},{\"tipo\":\"presion\"}]','mesh/data/840d8e377928','mesh/cmd/840d8e377928');
/*!40000 ALTER TABLE `nodos` ENABLE KEYS */;
UNLOCK TABLES;
COMMIT;
SET AUTOCOMMIT=@OLD_AUTOCOMMIT;

--
-- Table structure for table `pressure_measurements`
--

DROP TABLE IF EXISTS `pressure_measurements`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8mb4 */;
CREATE TABLE `pressure_measurements` (
  `id` int(11) NOT NULL AUTO_INCREMENT,
  `nodo` varchar(50) NOT NULL,
  `valor` float NOT NULL,
  `unidad` varchar(10) DEFAULT 'hPa',
  `relay` int(11) DEFAULT NULL,
  `mode` varchar(50) DEFAULT NULL,
  `layer` int(11) DEFAULT NULL,
  `fecha` timestamp NULL DEFAULT current_timestamp(),
  PRIMARY KEY (`id`)
) ENGINE=InnoDB AUTO_INCREMENT=31 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_uca1400_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `pressure_measurements`
--

SET @OLD_AUTOCOMMIT=@@AUTOCOMMIT, @@AUTOCOMMIT=0;
LOCK TABLES `pressure_measurements` WRITE;
/*!40000 ALTER TABLE `pressure_measurements` DISABLE KEYS */;
INSERT INTO `pressure_measurements` VALUES
(1,'nodo_bme_rele_1',1007.54,'hPa',0,'MANUAL_OFF',2,'2026-04-28 10:58:16'),
(2,'nodo_bme_rele_1',1007.48,'hPa',0,'MANUAL_OFF',2,'2026-04-28 10:58:46'),
(3,'nodo_bme_rele_1',1007.6,'hPa',0,'MANUAL_OFF',2,'2026-04-28 10:59:17'),
(4,'nodo_bme_rele_1',1007.49,'hPa',0,'MANUAL_OFF',2,'2026-04-28 10:59:47'),
(5,'nodo_bme_rele_1',1007.56,'hPa',0,'MANUAL_OFF',2,'2026-04-28 11:00:17'),
(6,'nodo_bme_rele_1',1007.53,'hPa',0,'MANUAL_OFF',2,'2026-04-28 11:00:48'),
(7,'nodo_bme_rele_1',1007.56,'hPa',0,'MANUAL_OFF',2,'2026-04-28 11:01:18'),
(8,'nodo_bme_rele_1',1007.57,'hPa',0,'MANUAL_OFF',2,'2026-04-28 11:01:48'),
(9,'nodo_bme_rele_1',1007.64,'hPa',0,'MANUAL_OFF',2,'2026-04-28 11:02:19'),
(10,'nodo_bme_rele_1',1007.56,'hPa',0,'MANUAL_OFF',2,'2026-04-28 11:02:49'),
(11,'nodo_bme_rele_1',1007.6,'hPa',0,'MANUAL_OFF',2,'2026-04-28 11:03:19'),
(12,'nodo_bme_rele_1',1007.68,'hPa',0,'MANUAL_OFF',2,'2026-04-28 11:03:50'),
(13,'nodo_bme_rele_1',1007.58,'hPa',0,'MANUAL_OFF',2,'2026-04-28 11:04:21'),
(14,'nodo_bme_rele_1',1007.57,'hPa',0,'MANUAL_OFF',2,'2026-04-28 11:04:51'),
(15,'nodo_bme_rele_1',1007.56,'hPa',0,'MANUAL_OFF',2,'2026-04-28 11:05:21'),
(16,'nodo_bme_rele_1',1007.62,'hPa',0,'MANUAL_OFF',2,'2026-04-28 11:05:52'),
(17,'nodo_bme_rele_1',1007.49,'hPa',0,'MANUAL_OFF',2,'2026-04-28 11:06:22'),
(18,'nodo_bme_rele_1',1007.55,'hPa',0,'MANUAL_OFF',2,'2026-04-28 11:06:52'),
(19,'nodo_bme_rele_1',1007.54,'hPa',0,'MANUAL_OFF',2,'2026-04-28 11:07:22'),
(20,'nodo_bme_rele_1',1007.59,'hPa',0,'MANUAL_OFF',2,'2026-04-28 11:07:53'),
(21,'nodo_bme_rele_1',1007.58,'hPa',0,'MANUAL_OFF',2,'2026-04-28 11:08:23'),
(22,'nodo_bme_rele_1',1007.58,'hPa',0,'MANUAL_OFF',2,'2026-04-28 11:08:54'),
(23,'nodo_bme_rele_1',1007.53,'hPa',0,'MANUAL_OFF',2,'2026-04-28 11:09:24'),
(24,'nodo_bme_rele_1',1007.59,'hPa',0,'MANUAL_OFF',2,'2026-04-28 11:09:55'),
(25,'nodo_bme_rele_1',1007.59,'hPa',0,'MANUAL_OFF',2,'2026-04-28 11:09:55'),
(26,'nodo_bme_rele_1',1007.63,'hPa',0,'MANUAL_OFF',2,'2026-04-28 11:10:24'),
(27,'nodo_bme_rele_1',1007.56,'hPa',0,'MANUAL_OFF',2,'2026-04-28 11:10:55'),
(28,'nodo_bme_rele_1',1007.58,'hPa',0,'MANUAL_OFF',2,'2026-04-28 11:11:25'),
(29,'nodo_bme_rele_1',1007.64,'hPa',0,'MANUAL_OFF',2,'2026-04-28 11:11:55'),
(30,'nodo_bme_rele_1',1007.49,'hPa',0,'MANUAL_OFF',2,'2026-04-28 11:12:26');
/*!40000 ALTER TABLE `pressure_measurements` ENABLE KEYS */;
UNLOCK TABLES;
COMMIT;
SET AUTOCOMMIT=@OLD_AUTOCOMMIT;

--
-- Table structure for table `temperature_measurements`
--

DROP TABLE IF EXISTS `temperature_measurements`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8mb4 */;
CREATE TABLE `temperature_measurements` (
  `id` int(11) NOT NULL AUTO_INCREMENT,
  `nodo` varchar(50) NOT NULL,
  `valor` float NOT NULL,
  `unidad` varchar(10) DEFAULT 'C',
  `relay` int(11) DEFAULT NULL,
  `mode` varchar(50) DEFAULT NULL,
  `layer` int(11) DEFAULT NULL,
  `fecha` timestamp NULL DEFAULT current_timestamp(),
  PRIMARY KEY (`id`)
) ENGINE=InnoDB AUTO_INCREMENT=31 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_uca1400_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `temperature_measurements`
--

SET @OLD_AUTOCOMMIT=@@AUTOCOMMIT, @@AUTOCOMMIT=0;
LOCK TABLES `temperature_measurements` WRITE;
/*!40000 ALTER TABLE `temperature_measurements` DISABLE KEYS */;
INSERT INTO `temperature_measurements` VALUES
(1,'nodo_bme_rele_1',25.28,'C',0,'MANUAL_OFF',2,'2026-04-28 10:58:16'),
(2,'nodo_bme_rele_1',25.26,'C',0,'MANUAL_OFF',2,'2026-04-28 10:58:46'),
(3,'nodo_bme_rele_1',25.26,'C',0,'MANUAL_OFF',2,'2026-04-28 10:59:17'),
(4,'nodo_bme_rele_1',25.28,'C',0,'MANUAL_OFF',2,'2026-04-28 10:59:47'),
(5,'nodo_bme_rele_1',25.31,'C',0,'MANUAL_OFF',2,'2026-04-28 11:00:17'),
(6,'nodo_bme_rele_1',25.33,'C',0,'MANUAL_OFF',2,'2026-04-28 11:00:48'),
(7,'nodo_bme_rele_1',25.31,'C',0,'MANUAL_OFF',2,'2026-04-28 11:01:18'),
(8,'nodo_bme_rele_1',25.32,'C',0,'MANUAL_OFF',2,'2026-04-28 11:01:48'),
(9,'nodo_bme_rele_1',25.31,'C',0,'MANUAL_OFF',2,'2026-04-28 11:02:19'),
(10,'nodo_bme_rele_1',25.31,'C',0,'MANUAL_OFF',2,'2026-04-28 11:02:49'),
(11,'nodo_bme_rele_1',25.33,'C',0,'MANUAL_OFF',2,'2026-04-28 11:03:19'),
(12,'nodo_bme_rele_1',25.33,'C',0,'MANUAL_OFF',2,'2026-04-28 11:03:50'),
(13,'nodo_bme_rele_1',25.34,'C',0,'MANUAL_OFF',2,'2026-04-28 11:04:21'),
(14,'nodo_bme_rele_1',25.34,'C',0,'MANUAL_OFF',2,'2026-04-28 11:04:21'),
(15,'nodo_bme_rele_1',25.33,'C',0,'MANUAL_OFF',2,'2026-04-28 11:04:51'),
(16,'nodo_bme_rele_1',25.31,'C',0,'MANUAL_OFF',2,'2026-04-28 11:05:21'),
(17,'nodo_bme_rele_1',25.31,'C',0,'MANUAL_OFF',2,'2026-04-28 11:05:52'),
(18,'nodo_bme_rele_1',25.32,'C',0,'MANUAL_OFF',2,'2026-04-28 11:06:22'),
(19,'nodo_bme_rele_1',25.3,'C',0,'MANUAL_OFF',2,'2026-04-28 11:06:52'),
(20,'nodo_bme_rele_1',25.3,'C',0,'MANUAL_OFF',2,'2026-04-28 11:07:22'),
(21,'nodo_bme_rele_1',25.28,'C',0,'MANUAL_OFF',2,'2026-04-28 11:07:53'),
(22,'nodo_bme_rele_1',25.27,'C',0,'MANUAL_OFF',2,'2026-04-28 11:08:23'),
(23,'nodo_bme_rele_1',25.27,'C',0,'MANUAL_OFF',2,'2026-04-28 11:08:54'),
(24,'nodo_bme_rele_1',25.27,'C',0,'MANUAL_OFF',2,'2026-04-28 11:09:24'),
(25,'nodo_bme_rele_1',25.28,'C',0,'MANUAL_OFF',2,'2026-04-28 11:09:54'),
(26,'nodo_bme_rele_1',25.26,'C',0,'MANUAL_OFF',2,'2026-04-28 11:10:24'),
(27,'nodo_bme_rele_1',25.27,'C',0,'MANUAL_OFF',2,'2026-04-28 11:10:55'),
(28,'nodo_bme_rele_1',25.27,'C',0,'MANUAL_OFF',2,'2026-04-28 11:11:25'),
(29,'nodo_bme_rele_1',25.27,'C',0,'MANUAL_OFF',2,'2026-04-28 11:11:55'),
(30,'nodo_bme_rele_1',25.28,'C',0,'MANUAL_OFF',2,'2026-04-28 11:12:26');
/*!40000 ALTER TABLE `temperature_measurements` ENABLE KEYS */;
UNLOCK TABLES;
COMMIT;
SET AUTOCOMMIT=@OLD_AUTOCOMMIT;
/*!40103 SET TIME_ZONE=@OLD_TIME_ZONE */;

/*!40101 SET SQL_MODE=@OLD_SQL_MODE */;
/*!40014 SET FOREIGN_KEY_CHECKS=@OLD_FOREIGN_KEY_CHECKS */;
/*!40014 SET UNIQUE_CHECKS=@OLD_UNIQUE_CHECKS */;
/*!40101 SET CHARACTER_SET_CLIENT=@OLD_CHARACTER_SET_CLIENT */;
/*!40101 SET CHARACTER_SET_RESULTS=@OLD_CHARACTER_SET_RESULTS */;
/*!40101 SET COLLATION_CONNECTION=@OLD_COLLATION_CONNECTION */;
/*M!100616 SET NOTE_VERBOSITY=@OLD_NOTE_VERBOSITY */;

-- Dump completed on 2026-06-10  8:46:05
